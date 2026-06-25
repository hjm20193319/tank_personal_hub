import math
import unittest

from shooting_module import (
    barrel_angle_is_possible,
    calculate_barrel_vertical_angle,
    calculate_horizontal_distance,
    calculate_turret_rf_command,
    ShootingAimer,
)


class ShootingModuleTests(unittest.TestCase):
    def test_horizontal_distance_uses_tank_and_obstacle_xz_coordinates(self):
        distance = calculate_horizontal_distance(
            {"x": 10, "z": 20},
            {"x": 13, "z": 24},
        )

        self.assertEqual(distance, 5.0)

    def test_barrel_angle_uses_requested_ballistic_formula(self):
        tank_position = {"x": 0.0, "y": 10.0, "z": 0.0}
        obstacle_average = {"x": 100.0, "y": 14.0, "z": 0.0}
        speed = 100.0
        gravity = 9.81
        horizontal_distance = 100.0
        delta_y = 14.0 - (10.0 + 2.56)
        speed_squared = speed * speed
        expected = math.atan(
            (
                speed_squared
                - math.sqrt(
                    speed_squared * speed_squared
                    - gravity
                    * (
                        gravity * horizontal_distance * horizontal_distance
                        + 2.0 * delta_y * speed_squared
                    )
                )
            )
            / (gravity * horizontal_distance)
        )

        actual = calculate_barrel_vertical_angle(
            tank_position,
            obstacle_average,
            projectile_speed=speed,
        )

        self.assertAlmostEqual(actual, expected)

    def test_turret_rf_moves_up_or_down_until_within_tolerance(self):
        up = calculate_turret_rf_command(
            current_turret_y=1.0,
            target_angle_degrees=2.0,
        )
        down = calculate_turret_rf_command(
            current_turret_y=3.0,
            target_angle_degrees=2.0,
        )
        aligned = calculate_turret_rf_command(
            current_turret_y=2.04,
            target_angle_degrees=2.0,
            tolerance_degrees=0.05,
        )

        self.assertEqual(up["turretRF"]["command"], "R")
        self.assertEqual(down["turretRF"]["command"], "F")
        self.assertTrue(aligned["is_aligned"])
        self.assertEqual(aligned["turretRF"], {"command": "", "weight": 0.0})

    def test_aimer_reports_alignment_once_until_alignment_is_lost(self):
        messages = []
        aimer = ShootingAimer(
            projectile_speed=100.0,
            tolerance_degrees=0.1,
            logger=messages.append,
        )
        tank_position = {"x": 0.0, "y": 0.0, "z": 0.0}
        obstacle_average = {"x": 100.0, "y": 2.56, "z": 0.0}
        target_angle = math.degrees(
            calculate_barrel_vertical_angle(
                tank_position,
                obstacle_average,
                projectile_speed=100.0,
            )
        )

        first = aimer.build_command(
            tank_position,
            obstacle_average,
            current_turret_y=target_angle,
        )
        second = aimer.build_command(
            tank_position,
            obstacle_average,
            current_turret_y=target_angle,
        )

        self.assertTrue(first["is_aligned"])
        self.assertTrue(second["is_aligned"])
        self.assertEqual(messages, ["조준 완료"])


    def test_barrel_angle_must_be_between_minus_5_and_10_degrees(self):
        self.assertTrue(barrel_angle_is_possible(-5.0))
        self.assertTrue(barrel_angle_is_possible(10.0))
        self.assertFalse(barrel_angle_is_possible(-5.01))
        self.assertFalse(barrel_angle_is_possible(10.01))

    def test_aimer_reports_impossible_when_angle_is_outside_range(self):
        messages = []
        aimer = ShootingAimer(
            projectile_speed=100.0,
            logger=messages.append,
        )

        first = aimer.build_command(
            tank_position={"x": 0.0, "y": 0.0, "z": 0.0},
            obstacle_average={"x": 1.0, "y": 100.0, "z": 0.0},
            current_turret_y=0.0,
        )
        second = aimer.build_command(
            tank_position={"x": 0.0, "y": 0.0, "z": 0.0},
            obstacle_average={"x": 1.0, "y": 100.0, "z": 0.0},
            current_turret_y=0.0,
        )

        self.assertFalse(first["is_possible"])
        self.assertFalse(second["is_possible"])
        self.assertEqual(first["turretRF"], {"command": "", "weight": 0.0})
        self.assertGreater(first["target_angle_degrees"], 10.0)
        self.assertEqual(messages, ["사격 불가능"])


if __name__ == "__main__":
    unittest.main()
