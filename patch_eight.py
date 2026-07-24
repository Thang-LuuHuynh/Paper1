import re

with open('/home/thang/Paper1/Paper1-main/src/amr_control/amr_control/bsmc_eight.py', 'r') as f:
    code = f.read()

# Fix duplicates if any
code = code.replace("self.declare_parameter('v_cmd_scale', 1.0)\n        self.declare_parameter('v_cmd_scale', 1.0)", "self.declare_parameter('v_cmd_scale', 1.0)")

# Add i_y variables
if "self.i_y = 0.0" not in code:
    code = code.replace("self.yaw_feedforward_lpf = 0.0", 
"""self.yaw_feedforward_lpf = 0.0
        self.i_y = 0.0
        self.KI_Y = float(self.get_parameter('ki_y').value)
        self.I_Y_MAX = float(self.get_parameter('i_y_max').value)""")

# Add w_iy logic
if "w_iy" not in code:
    old_w = """        w_ey = v_feedback * k2_control * e_y
        w_eth = v_feedback * k3_control * math.sin(e_theta)
        w_smc = self.Ks2 * sat_s2
        
        w_cmd = (
            entry_scale * w_ff_scale * w_d
            + w_ey + w_eth
            + w_smc
            + self.yaw_feedforward
        )
        
        if distance_from_center < 0.2 and trajectory_t > 5.0:
            self.get_logger().info(f'[Center] ey: {e_y:.3f} | etheta: {e_theta:.3f} || w_ey: {w_ey:.3f} | w_eth: {w_eth:.3f} | w_smc: {w_smc:.3f}')"""

    new_w = """        w_ey = v_feedback * k2_control * e_y
        w_eth = v_feedback * k3_control * math.sin(e_theta)
        w_smc = self.Ks2 * sat_s2
        
        self.i_y += e_y * self.timer_period
        self.i_y = max(-self.I_Y_MAX, min(self.I_Y_MAX, self.i_y))
        w_iy = self.KI_Y * self.i_y
        
        w_cmd = (
            entry_scale * w_ff_scale * w_d
            + w_ey + w_eth
            + w_smc
            + w_iy
            + self.yaw_feedforward
        )
        
        if distance_from_center < 0.2 and trajectory_t > 5.0:
            self.get_logger().info(f'[Center] ey: {e_y:.3f} | etheta: {e_theta:.3f} || w_ey: {w_ey:.3f} | w_eth: {w_eth:.3f} | w_iy: {w_iy:.3f}')"""
    
    code = code.replace(old_w, new_w)

with open('/home/thang/Paper1/Paper1-main/src/amr_control/amr_control/bsmc_eight.py', 'w') as f:
    f.write(code)
