from library import *
from globals import *

class FuzzyControl:
    def __init__(self, max_lf=13):
        self.max_lf = max_lf
        self.init_fuzzy_system()

    def init_fuzzy_system(self):
        self.yaw = ctrl.Antecedent(np.arange(-100, 101, 0.01), 'yaw')
        self.yaw['negative'] = fuzz.trimf(self.yaw.universe, [-100, -100, -3])
        self.yaw['normal'] = fuzz.trimf(self.yaw.universe, [-3, 0, 3])
        self.yaw['positive'] = fuzz.trimf(self.yaw.universe, [3, 100, 100])
        
        self.adjustment_femur = ctrl.Consequent(np.arange(-self.max_lf, self.max_lf + 1, 1), 'adjustment_femur')
        self.adjustment_femur['low'] = fuzz.trimf(self.adjustment_femur.universe, [-self.max_lf, -lf, 0])
        self.adjustment_femur['normal'] = fuzz.trimf(self.adjustment_femur.universe, [-lf, 0, lf])
        self.adjustment_femur['high'] = fuzz.trimf(self.adjustment_femur.universe, [0, lf, self.max_lf])
        
        rule1 = ctrl.Rule(self.yaw['negative'], self.adjustment_femur['high'])
        rule2 = ctrl.Rule(self.yaw['normal'], self.adjustment_femur['normal'])
        rule3 = ctrl.Rule(self.yaw['positive'], self.adjustment_femur['low'])
        
        self.robot_stability_ctrl = ctrl.ControlSystem([rule1, rule2, rule3])
        self.robot_stability = ctrl.ControlSystemSimulation(self.robot_stability_ctrl)

    def compute_adjustment(self, yaw_value):
        self.robot_stability.input['yaw'] = yaw_value
        self.robot_stability.compute()
        return round(self.robot_stability.output['adjustment_femur'].item())

    def apply_adjustment(self, yaw_value):
        adjustment_femur_value = self.compute_adjustment(yaw_value)
        print(f"Adjustment femur: {adjustment_femur_value}")
        return adjustment_femur_value
