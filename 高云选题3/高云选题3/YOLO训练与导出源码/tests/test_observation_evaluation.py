import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("evaluation", Path(__file__).resolve().parents[1] / "tools/evaluate_observation.py")
evaluation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(evaluation)


class EvaluationTests(unittest.TestCase):
    def test_duplicate_prediction_matches_only_once(self):
        gt = [{"bbox_xyxy": [0, 0, 10, 10], "class_id": 1}]
        pred = [dict(gt[0], confidence=.9), dict(gt[0], confidence=.7)]
        statuses, tps = evaluation.match_predictions(gt, pred)
        self.assertEqual(tps, [0])
        self.assertEqual(statuses[0]["status"], "matched")

    def test_wrong_class_is_confusion_not_true_positive(self):
        gt = [{"bbox_xyxy": [0, 0, 10, 10], "class_id": 1}]
        pred = [{"bbox_xyxy": [0, 0, 10, 10], "class_id": 2, "confidence": .9}]
        statuses, tps = evaluation.match_predictions(gt, pred)
        self.assertEqual(tps, [])
        self.assertEqual(statuses[0]["status"], "confused")

    def test_one_wrong_prediction_cannot_explain_two_people(self):
        gt = [{"bbox_xyxy": [0, 0, 10, 10], "class_id": 1}] * 2
        pred = [{"bbox_xyxy": [0, 0, 10, 10], "class_id": 2, "confidence": .9}]
        statuses, _ = evaluation.match_predictions(gt, pred)
        self.assertEqual(sorted(s["status"] for s in statuses), ["confused", "missed"])

    def test_distance_and_size_boundaries(self):
        self.assertEqual(evaluation.distance_bin(None), "unknown")
        self.assertEqual(evaluation.distance_bin(.4), "0.4-0.6m")
        self.assertEqual(evaluation.distance_bin(1), "0.6-1.0m")
        self.assertEqual(evaluation.angle_bin(20), "20-40deg")
        self.assertTrue(evaluation.priority_observable({"distance_m": 1, "height_px": 40, "short_side_px": 12}))
        self.assertFalse(evaluation.priority_observable({"distance_m": None, "height_px": 40, "short_side_px": 12}))

    def test_geometry_uses_instance_not_capture_target(self):
        item = {"class_id": 1, "source": {"node": "wall", "source_instance": "firefighter"}}
        geometry = {("wall", 0, "firefighter"): {"position": [1, 0, 0], "normal": [1, 0, 0], "class_id": 1}}
        row = {"camera": {"position_world": [0, 0, 0]}, "capture_plan": {"sampling_distance_m": 99}}
        result = evaluation.instance_geometry(item, row, geometry)
        self.assertAlmostEqual(result["distance_m"], 1)
        self.assertAlmostEqual(result["obliquity_deg"], 0)
        self.assertEqual(evaluation.instance_geometry(item, row, {})["geometry_status"], "unknown")

    def test_size_only_retains_unknown_geometry_and_zero_bin_is_none(self):
        obj = {"class_id": 1, "status": "matched", "height_px": 44, "short_side_px": 15,
               "distance_m": None, "geometry_status": "unknown", "distance_bin": "unknown",
               "angle_bin": "unknown", "size_bin": "height>=40,12<=short<16px", "lighting_bin": "neutral"}
        report = evaluation.summary([obj], [{"predictions": [{"class_id": 1}]}])
        rows = report["bins"]["observation"]
        size = next(r for r in rows if r["bin"] == "size_only_h40_s12" and r["class"] == "firefighter")
        near = next(r for r in rows if r["bin"] == "distance_0.4-1.0m_h40_s12" and r["class"] == "firefighter")
        self.assertEqual(size["recall"], 1)
        self.assertEqual(near["gt"], 0)
        self.assertIsNone(near["recall"])

    def test_playback_is_selected_by_metadata_with_light_pairs(self):
        rows=[]
        strata=[("frontal_0_20",False),("frontal_0_20",True),("oblique_20_40",False),
                ("oblique_40_55",False),("oblique_40_55",True),("oblique_20_40",False)]
        for i,(angle,occluded) in enumerate(strata):
            for light in ["neutral","cool_dim"]:
                rows.append({"split":"train","source_version":"v8","variant":"original","eye":"L",
                    "sample_id":f"{i}_{light}","pose_group":str(i),
                    "capture_plan":{"sampling_target":{"class_id":1},"angle_bin":angle,
                                    "occluder":{"board":True} if occluded else None,"lighting_profile":light}})
        result=evaluation.select_v8_playback(list(reversed(rows)))
        self.assertEqual(len(result),12)
        self.assertEqual(len({r['pose_group'] for r in result}),6)
        self.assertEqual(result,evaluation.select_v8_playback(rows))


if __name__ == "__main__":
    unittest.main()
