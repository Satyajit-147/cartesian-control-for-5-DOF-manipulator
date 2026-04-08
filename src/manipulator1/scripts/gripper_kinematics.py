#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import numpy as np

class GripperKinematics(Node):
    def __init__(self):
        super().__init__('gripper_kinematics_node')

        # === CONFIGURATION START ===
        # List of the 3 ACTUATED gripper joint names (User Verified)
        self.actuated_joint_names = [
            "link3_j_bevel1",
            "bevel1_j_bevel_coupler",
            "servo_j_gl1" # Verified Active driver
        ]

        # List of PASSIVE gripper joint names (User Verified List)
        # Excludes arm joints & the 3 active joints above.
        self.passive_joint_names = sorted(list(set([
            # "bevel1_j_bevel_coupler", # This is ACTIVE
            "bevel_coupler_j_bevel2", # Needs formula based on active coupler
            "bevel_coupler_j_bevel3", # Needs formula based on active bevel1
            "g_main_body_j_gl4M",   # Output from 4-bar (theta4 left)
            "gl4M_jaw_cover2",      # Needs formula based on gl4M / theta4_left
            "g_mainbody_j_gl4",     # Output from 4-bar (theta4 right) - Check URDF for exact name 'g_mainbody' vs 'g_main_body'
            "gl4_j_jaw_cover1",     # Needs formula based on gl4 / theta4_right
            "gl1_j_gl2",            # Input to 4-bar (theta2) - Needs formula based on servo_j_gl1
            "gl2_j_gl3",            # Output from 4-bar (theta3 right)
            "gl3_j_jaw_cover",      # Needs formula based on gl3 / theta3_right
            "gl1_j_gl2M",           # Mirrored Input to 4-bar? Needs formula based on servo_j_gl1
            "gl2M_j_gl3M",          # Output from 4-bar (theta3 left)
            "gl3M_j_jaw_cover_2",   # Needs formula based on gl3M / theta3_left
            # Add other known passive joints from URDF if missing from list above
            "jaw1_j_jaw11",         # Example if exists
            "jaw2_j_jaw22",         # Example if exists
            "g_main_body_j_jaw1",   # Example if exists - Needs formula based on theta4?
            "g_main_body_j_jaw2",   # Example if exists - Needs formula based on theta4?
            "jaw1_j_jaw_cover",     # Example if exists
            "jaw2_j_jaw_cover2",     # Example if exists
            "jaw_cover_1_jaw1",    # Example if exists
            "jaw_cover_2_jaw2",    # Example if exists
            "bevel2_j_outer_shaft1",# Example if exists - Needs formula based on bevel_coupler_j_bevel2
            "bevel3_j_outer_shaft2",# Example if exists - Needs formula based on bevel_coupler_j_bevel3
            "gl3_j_gl4",            # Example if exists - Needs formula based on theta4_right?
            "gl3M_j_gl4M",          # Example if exists - Needs formula based on theta4_left?
            # ---> Double check URDF against this final list <---
        ])))

        # Ensure no active joints accidentally included in passive list
        for active_joint in self.actuated_joint_names:
            if active_joint in self.passive_joint_names:
                self.get_logger().warn(f"Joint '{active_joint}' incorrectly listed as passive. Removing.")
                self.passive_joint_names.remove(active_joint)

        self.get_logger().info(f"Using ACTIVE gripper joints: {self.actuated_joint_names}")
        self.get_logger().info(f"Derived PASSIVE gripper joints ({len(self.passive_joint_names)}): {self.passive_joint_names}")

        # Link lengths from image_ec8e59.png (meters)
        self.l1 = 0.038599
        self.l2 = 0.018682 # Length of link driven by theta2 (likely gl1)
        self.l3 = 0.032000 # Length of link corresponding to theta3 (likely gl2 or gl3?)
        self.l4 = 0.020676 # Length of link corresponding to theta4 (likely gl4?)

        # Map formula outputs to joint names (VERIFY THESE MAPPINGS!)
        self.theta2_joint_name_right = "gl1_j_gl2"          # Input angle to 4-bar (derived from servo)
        self.theta3_joint_name_right = "gl2_j_gl3"          # Output angle from 4-bar (right)
        self.theta4_joint_name_right = "g_mainbody_j_gl4"   # Output angle from 4-bar (right) - CHECK NAME 'g_mainbody' vs 'g_main_body' in URDF
        # Mirrored side mappings (ASSUMING SIMPLE MIRRORING - VERIFY!)
        self.theta2_joint_name_left = "gl1_j_gl2M"          # Mirrored Input? Or maybe follows right side?
        self.theta3_joint_name_left = "gl2M_j_gl3M"         # Mirrored Output (right * sign?)
        self.theta4_joint_name_left = "g_main_body_j_gl4M"  # Mirrored Output (right * sign?)

        # === CONFIGURATION END ===

        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.passive_joint_state_pub = self.create_publisher(JointState, '/passive_joint_states', 10)
        self.actuated_positions = {}
        self.get_logger().info('Gripper Kinematics Node Started')

    def joint_state_callback(self, msg: JointState):
        # ... (Same logic to update self.actuated_positions for the 3 active joints) ...
        updated = False; temp_positions = self.actuated_positions.copy()
        for i, name in enumerate(msg.name):
            if name in self.actuated_joint_names:
                new_position = msg.position[i]
                if name not in temp_positions or not math.isclose(new_position, temp_positions[name], rel_tol=1e-4):
                    temp_positions[name] = new_position; updated = True
        if updated and len(temp_positions) == len(self.actuated_joint_names):
             self.actuated_positions = temp_positions
             self.calculate_and_publish_passive_states()

    def calculate_four_bar(self, theta2):
        # ... (Same robust four-bar calculation function as before) ...
        theta2_rad = theta2
        try:
            A = math.sin(theta2_rad); B = math.cos(theta2_rad) - (self.l1 / self.l2)
            C = (-self.l1 / self.l4) * math.cos(theta2_rad) + (self.l1**2 + self.l2**2 - self.l3**2 + self.l4**2) / (2 * self.l2 * self.l4)
            sqrt_term_val = A**2 + B**2 - C**2
            if sqrt_term_val < 0:
                if abs(sqrt_term_val) < 1e-9: sqrt_term_val = 0.0
                else: self.get_logger().warn(f"4-bar sqrt neg: {sqrt_term_val:.4f}"); return None, None
            sqrt_term = math.sqrt(sqrt_term_val)
            numerator_plus = A + sqrt_term; denominator = B + C
            if math.isclose(denominator, 0.0, abs_tol=1e-9): self.get_logger().warn("4-bar denom zero"); return None, None
            theta3_sol_plus = 2 * math.atan2(numerator_plus, denominator)
            # theta3_sol_minus = 2 * math.atan2(A - sqrt_term, denominator) # If needed
            theta3 = theta3_sol_plus # <<< VERIFY THIS CHOICE (+ or -)
            y_comp = self.l2 * math.sin(theta2_rad) - self.l3 * math.sin(theta3)
            x_comp = self.l1 + self.l2 * math.cos(theta2_rad) - self.l3 * math.cos(theta3)
            theta4 = math.atan2(y_comp, x_comp)
            return theta3, theta4
        except Exception as e: self.get_logger().error(f"4-bar calc error: {e}"); return None, None

    def calculate_and_publish_passive_states(self):
        if len(self.actuated_positions) != len(self.actuated_joint_names): return # Wait

        # Get the current ACTIVE positions
        theta_actuated_bevel1 = self.actuated_positions.get("link3_j_bevel1", 0.0)
        theta_actuated_coupler = self.actuated_positions.get("bevel1_j_bevel_coupler", 0.0)
        theta_actuated_servo = self.actuated_positions.get("servo_j_gl1", 0.0) # Verified Active

        passive_positions_dict = {} # To store calculated passive positions

        # --- 1. Calculate Four-Bar Linkage Input (theta2) ---
        # ****** YOU MUST VERIFY THIS RELATIONSHIP ******
        # How does the servo angle relate to the angle of the l2 link (gl1_j_gl2)?
        # ASSUMPTION: Direct 1:1 relationship for now. Replace if needed.
        # This calculates the position for the now-passive gl1_j_gl2 joint
        theta2_input_right = theta_actuated_servo * 1.0 # Placeholder: Direct drive? Gearing? Offset?
        passive_positions_dict[self.theta2_joint_name_right] = theta2_input_right

        # How does the left side l2 link (gl1_j_gl2M) relate? Mirror servo? Same?
        # ASSUMPTION: Simple mirroring. Replace if needed.
        theta2_input_left = -theta_actuated_servo * 1.0 # Placeholder mirror <<< VERIFY
        passive_positions_dict[self.theta2_joint_name_left] = theta2_input_left

        # --- 2. Calculate Four-Bar Linkage Outputs (theta3, theta4) ---
        theta3_calc_right, theta4_calc_right = self.calculate_four_bar(theta2_input_right)
        theta3_calc_left, theta4_calc_left = self.calculate_four_bar(theta2_input_left) # Use mirrored input

        # Handle calculation failure for right side
        if theta3_calc_right is None:
             self.get_logger().warn("Failed RIGHT 4-bar calc, using zeros.", throttle_duration_sec=2.0)
             theta3_calc_right, theta4_calc_right = 0.0, 0.0 # Use zeros on failure
        passive_positions_dict[self.theta3_joint_name_right] = theta3_calc_right
        passive_positions_dict[self.theta4_joint_name_right] = theta4_calc_right

        # Handle calculation failure for left side
        if theta3_calc_left is None:
             self.get_logger().warn("Failed LEFT 4-bar calc, using zeros.", throttle_duration_sec=2.0)
             theta3_calc_left, theta4_calc_left = 0.0, 0.0 # Use zeros on failure
        # Assign left side outputs - <<< VERIFY MIRRORING/SIGN from calculate_four_bar results
        passive_positions_dict[self.theta3_joint_name_left] = theta3_calc_left # Sign might need flipping depending on URDF definition
        passive_positions_dict[self.theta4_joint_name_left] = theta4_calc_left # Sign might need flipping

        # --- 3. Calculate OTHER Passive Joints ---
        # ****** YOU MUST IMPLEMENT THESE FORMULAS ******
        # Replace all placeholder logic below.

        # Example: Bevel gears might depend on the active bevel inputs
        passive_positions_dict["bevel_coupler_j_bevel3"] = theta_actuated_bevel1 * 1.0 # Placeholder (1:1?)
        passive_positions_dict["bevel_coupler_j_bevel2"] = theta_actuated_coupler * 1.0 # Placeholder (1:1?)

        # Example: Secondary gl links might follow primary ones (theta3 outputs)
        passive_positions_dict["gl3_j_gl4"] = theta4_calc_right * 1.0 # Placeholder relationship to theta4
        passive_positions_dict["gl3M_j_gl4M"] = theta4_calc_left * 1.0 # Placeholder relationship to theta4

        # Example: Jaw links might depend on final links (theta4 outputs)
        jaw1_joint = "g_main_body_j_jaw1" # Verify name
        jaw2_joint = "g_main_body_j_jaw2" # Verify name
        if jaw1_joint in self.passive_joint_names:
            passive_positions_dict[jaw1_joint] = theta4_calc_right + math.radians(10) # Placeholder offset
        if jaw2_joint in self.passive_joint_names:
             # Jaw 2 might depend on LEFT side theta4
             passive_positions_dict[jaw2_joint] = theta4_calc_left - math.radians(10) # Placeholder offset <<< VERIFY

        # Example: Jaw covers might depend on gl4/theta4 or main jaw angles
        passive_positions_dict["gl4_j_jaw_cover1"] = theta4_calc_right # Placeholder
        passive_positions_dict["gl4M_jaw_cover2"] = theta4_calc_left # Placeholder
        passive_positions_dict["gl3_j_jaw_cover"] = theta3_calc_right # Placeholder
        passive_positions_dict["gl3M_j_jaw_cover_2"] = theta3_calc_left # Placeholder

        # Fill remaining passive joints that haven't been calculated yet
        for name in self.passive_joint_names:
            if name not in passive_positions_dict:
                 # Add specific logic for sub-links (jaw11, jaw22) or others
                 passive_positions_dict[name] = 0.0 # Default placeholder if no formula

        # --- Create and Publish ---
        calculated_passive_positions = []
        for name in self.passive_joint_names: # Use the defined order
            calculated_passive_positions.append(passive_positions_dict.get(name, 0.0))

        if len(calculated_passive_positions) != len(self.passive_joint_names): return # Error

        passive_msg = JointState()
        passive_msg.header.stamp = self.get_clock().now().to_msg()
        passive_msg.name = self.passive_joint_names
        passive_msg.position = calculated_passive_positions
        self.passive_joint_state_pub.publish(passive_msg)

# ... (main function same as before) ...
def main(args=None):
    rclpy.init(args=args)
    node = GripperKinematics()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()	
