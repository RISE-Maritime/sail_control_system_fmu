pub use fmu_from_struct::prelude::*;
use std::f64::consts::PI;

fn wrap_angle_pi_pi(angle: f64) -> f64 {
    let mut output = angle;

    while output > PI {
        output -= 2.0 * PI;
    }
    while output < -PI {
        output += 2.0 * PI;
    }
    output
}

#[derive(Debug, Default, Clone, Fmu)]
#[fmi_version = 3]
pub struct SailControlSystem {
    #[parameter]
    pub kp: f64,

    #[input]
    pub apparent_wind_speed: f64,
    pub apparent_wind_angle: f64,
    pub sail_angle: f64,
    pub aoa_setpoint: f64,

    #[output]
    pub sail_angle_order: f64,
    pub error: f64,
}

impl FmuFunctions for SailControlSystem {
    fn do_step(&mut self, _current_time: f64, _time_step: f64) {
        // Very simple integrating controller
        let awa = wrap_angle_pi_pi(self.apparent_wind_angle);
        let aoa_actual = wrap_angle_pi_pi(awa - wrap_angle_pi_pi(self.sail_angle));
        let aoa_wanted = wrap_angle_pi_pi(self.aoa_setpoint).copysign(awa);
        self.error = wrap_angle_pi_pi(aoa_wanted - aoa_actual);
        self.sail_angle_order = wrap_angle_pi_pi(self.sail_angle + self.kp * self.error);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use more_asserts::*;

    #[test]
    fn test_wrap_angle_pi_pi() {
        let result = wrap_angle_pi_pi(0.0);
        assert_eq!(result, 0.0);

        let result = wrap_angle_pi_pi(2.0 * PI);
        assert_eq!(result, 0.0);

        let result = wrap_angle_pi_pi(-2.0 * PI);
        assert_eq!(result, 0.0);
    }

    #[test]
    fn test_sail_control_system_do_step() {
        let mut scs = SailControlSystem {
            kp: -0.01,
            apparent_wind_speed: 0.0,
            apparent_wind_angle: 0.0,
            sail_angle: 0.0,
            aoa_setpoint: 18.0_f64.to_radians(),
            sail_angle_order: 0.0,
            error: 0.0,
        };

        // AWA to 90 deg, we should start rotating clock-wise
        scs.apparent_wind_angle = 90.0_f64.to_radians();
        scs.do_step(0.0, 0.1);
        assert_gt!(scs.sail_angle_order.to_degrees(), 0.0);

        // AWA to -90 deg, we should start rotating counter clock-wise
        scs.apparent_wind_angle = -90.0_f64.to_radians();
        scs.do_step(0.0, 0.1);
        assert_lt!(scs.sail_angle_order, 0.0);
    }
}
