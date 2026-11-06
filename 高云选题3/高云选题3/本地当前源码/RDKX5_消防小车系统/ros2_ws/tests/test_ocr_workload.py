import unittest
import numpy as np
from rescue_vision.building_ocr import bounded_text_boxes, OCRReader


class OCRWorkloadTests(unittest.TestCase):
    def test_quads_rescaled_to_original_geometry(self):
        boxes = [[[10, 20], [90, 20], [90, 40], [10, 40]]]
        result = bounded_text_boxes(boxes, (512, 435), (1280, 1088), 6)
        self.assertEqual(len(result), 1)
        np.testing.assert_allclose(result[0][:, 0], [25, 225, 225, 25])
        np.testing.assert_allclose(result[0][:, 1], np.array([20, 20, 40, 40])*1088/435, rtol=1e-6)

    def test_candidate_count_and_recognition_width_are_bounded(self):
        boxes = [np.array([[0, 0], [40+i, 0], [40+i, 20], [0, 20]], dtype=float) for i in range(30)]
        boxes += [np.array([[0, 0], [500, 0], [500, 1], [0, 1]], dtype=float)]
        result = bounded_text_boxes(boxes, (512, 512), (1280, 1280), 4)
        self.assertEqual(len(result), 4)
        self.assertEqual(result[0][1, 0], 69*2.5)

    def test_invalid_and_tiny_quads_do_not_reach_recognizer(self):
        boxes = [np.zeros((4, 2)), np.ones((3, 2)), np.full((4, 2), float('nan')),
                 [[0, 0], [5, 0], [5, 1], [0, 1]]]
        self.assertEqual(bounded_text_boxes(boxes, (512, 512), (1280, 1280), 6), [])

    def test_detector_resized_recognizer_keeps_source_image(self):
        image = np.zeros((1088, 1280, 3), dtype=np.uint8)
        observed = {}
        class Engine:
            def text_det(self, analysis):
                observed['analysis_shape'] = analysis.shape
                return np.array([[[10, 20], [90, 20], [90, 40], [10, 40]]]), .1
            def get_crop_img_list(self, source, boxes):
                observed['source_is_original'] = source is image
                return [source[50:100, 25:225]]
            def text_rec(self, crops):
                return [('01安居楼', .97)], .1
        reader = OCRReader.__new__(OCRReader)
        reader.engine = Engine()
        reader.analysis_max_side, reader.max_text_regions, reader.use_angle_cls = 512, 6, False
        result = reader.predict(image)
        self.assertEqual(observed['analysis_shape'], (435, 512, 3))
        self.assertTrue(observed['source_is_original'])
        self.assertEqual(result[0]['quad'][0][0], 25.)
        self.assertEqual(result[0]['text'], '01安居楼')
        self.assertTrue(reader.last_timings['original_recognition_pixels'])


if __name__ == '__main__':
    unittest.main()
