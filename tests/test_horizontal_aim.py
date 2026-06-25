import unittest

from horizontal_aim import HorizontalAimer


class HorizontalAimerTests(unittest.TestCase):
    def test_centered_bbox_stops_turret_and_reports_alignment(self):
        aimer = HorizontalAimer()

        command = aimer.build_command(
            bbox=[319, 0, 321, 100],
            image_width=640,
            now=1.0,
        )

        self.assertTrue(command["is_aligned"])
        self.assertEqual(command["turretQE"], {"command": "", "weight": 0.0})

    def test_left_bbox_turns_turret_left(self):
        aimer = HorizontalAimer()

        command = aimer.build_command(
            bbox=[100, 0, 120, 100],
            image_width=640,
            now=1.0,
        )

        self.assertEqual(command["turretQE"]["command"], "Q")
        self.assertGreater(command["turretQE"]["weight"], 0.0)

    def test_alignment_uses_wider_exit_margin_after_lock(self):
        aimer = HorizontalAimer()
        aimer.build_command(
            bbox=[319, 0, 321, 100],
            image_width=640,
            now=1.0,
        )

        command = aimer.build_command(
            bbox=[320.8, 0, 322.8, 100],
            image_width=640,
            now=1.1,
        )

        self.assertTrue(command["is_aligned"])
        self.assertEqual(command["turretQE"], {"command": "", "weight": 0.0})

    def test_no_target_command_resets_alignment_state(self):
        aimer = HorizontalAimer()
        aimer.build_command(
            bbox=[319, 0, 321, 100],
            image_width=640,
            now=1.0,
        )

        command = aimer.build_no_target_command(image_width=640)

        self.assertFalse(aimer.is_aligned)
        self.assertFalse(command["is_aligned"])
        self.assertEqual(command["bbox_center_x"], None)


if __name__ == "__main__":
    unittest.main()
