"""Regression checks for absence, confusion, and GT-conditioned denominators."""
import unittest

from evaluate_firefighter_holdout import enrich_instances, qualified, summarize


class HoldoutEvaluationTests(unittest.TestCase):
    def row(self, designated=6):
        return {"id":"frameL","pose_id":"p0","pair_id":"pair0","scene_id":"scene0",
                "eye":"L","lighting":"normal","occlusion":"none","designated_target_id":designated}

    def obj(self, cid=1):
        return {"instance_id":6,"class_id":cid,"bbox_xyxy":[10,20,30,70],
                "distance_m":.7,"angle_deg":20}

    def test_wrong_class_is_fn_and_fp_not_tp(self):
        preds=[{"class_id":2,"confidence":.8,"bbox_xyxy":[10,20,30,70]}]
        instances=enrich_instances(self.row(),{"instances":[self.obj()]},preds)
        stats=summarize(instances,[{"sample_id":"frameL","predictions":preds,"designated_target_id":6}])
        self.assertEqual(stats["classes"]["firefighter"]["tp"],0)
        self.assertEqual(stats["classes"]["firefighter"]["confused_as"],{"person":1})
        self.assertEqual(stats["classes"]["person"]["fp"],1)
        self.assertNotIn("precision",stats["bins"]["scope"][0])

    def test_absent_designated_target_is_error_not_silently_discarded(self):
        with self.assertRaises(ValueError):
            enrich_instances(self.row(),{"instances":[]},[])

    def test_negative_misclassification_is_counted(self):
        preds=[{"class_id":1,"confidence":.8,"bbox_xyxy":[10,20,30,70]}]
        instances=enrich_instances(self.row(None),{"instances":[self.obj(2)]},preds)
        stats=summarize(instances,[{"sample_id":"frameL","predictions":preds,"designated_target_id":None}])
        self.assertEqual(stats["negative_controls"],{"images":1,"firefighter_false_positive_images":1,"firefighter_false_positive_boxes":1})
        self.assertEqual(stats["classes"]["firefighter"]["precision"],0)

    def test_visible_pixels_and_true_range_define_qualification(self):
        item={"distance_m":.4,"angle_deg":55,"height_px":40,"short_side_px":12}
        self.assertTrue(qualified(item))
        self.assertTrue(qualified({**item,"distance_m":1}))
        self.assertFalse(qualified({**item,"distance_m":None}))
        self.assertFalse(qualified({**item,"height_px":39}))
        self.assertFalse(qualified({**item,"short_side_px":11}))
        self.assertFalse(qualified({**item,"angle_deg":56}))
        self.assertFalse(qualified({**item,"angle_deg":None}))


if __name__=="__main__":
    unittest.main()
