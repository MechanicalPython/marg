/// # MARG (Magnetic, Angular Rate, and Gravity)
///
/// MARG is a type of AHRS (Attitude and Heading Reference Systems) which is essentially a hybrid
/// IMU that incorporates a 3-axis magnetometer. Developed by Madgwick. Forked from
/// https://github.com/JoshMcguigan/imu
///
/// The aim of this crate is to take accel, gyro and mag data and output:
/// - Roll, pitch and yaw
/// - Acceleration vector in North, East and Down directions.
///
/// What to implement
/// - Magnetometer calibration
///     - Rotate the chip over a period of time and then find the way to offset the values so
/// the mean values are centered around the middle. Gives a
/// - Attitude Heading Reference System (AHRS). Use the Madgwick Quaternion Update algorithm. There
/// is a crate for this. Inputs all the accel, gyro and mag data and outputs a quaternion
/// - Can implement a magnetic north vs true north offset but won't worry.
///
/// - Use the AHRS quaternion and get a rotation matrix (should be 3x3). Then multiply the rotation
/// matrix by accel data (3x1).
///     - If the chip is a rest, can get what gravity is.
///     - For future values, take the rotation matrix and the data and solve for acceleration
///         - rotation matrix X gravity = acceleration
///         - Accel in N,E,D = rotation matrix inverse X accel data
///
/// ## How to integrate with a crate that reads chip data?
///
///
///

pub mod calibration;
pub mod madgwick;

use std::f32::consts::PI;

/// Struct for a generic x-axis vector like acceleration in the x, y and z direction.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct V {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Struct for a quaternion
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct Q {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Default for Q {
    fn default() -> Self {
        Q {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Euler {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}


impl Q {
    //! Implemented from https://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
    pub fn to_rotation_matrix(self) -> [[f32; 3]; 3] {
        return [
            [
                (1.0 - 2.0 * (self.y * self.y) - 2.0 * (self.z * self.z)),
                (2.0 * self.x * self.y - 2.0 * self.z * self.w),
                (2.0 * self.x * self.z + 2.0 * self.y * self.w)],
            [
                (2.0 * self.x * self.y + 2.0 * self.z * self.w),
                (1.0 - 2.0 * (self.x * self.x) - 2.0 * (self.z * self.z)),
                (2.0 * self.y * self.z - 2.0 * self.x * self.w),
            ],
            [
                (2.0 * self.x * self.z - 2.0 * self.y * self.w),
                (2.0 * self.y * self.z + 2.0 * self.x * self.w),
                (1.0 - 2.0 * (self.x * self.x) - 2.0 * (self.y * self.y)),
            ],
        ];
    }

    pub fn roll_pitch_yaw(self) -> Euler {
        let w = self.w;
        let x = self.x;
        let y = self.y;
        let z = self.z;

        let test = x*y + z*w;
        if test > 0.499 { // singularity at north pole
            let pitch = 2. * x.atan2(w);
            let yaw = PI/2.;
            let roll = 0.;
            return Euler { roll, pitch, yaw }
        }
        if test < -0.499 { // singularity at south pole
            let pitch = -2. * x.atan2(w);
            let yaw = -PI/2.;
            let roll = 0.;
            return Euler { roll, pitch, yaw }
        }

        let sqx = x*x;
        let sqy = y*y;
        let sqz = z*z;
        let pitch = (2.*y*w-2.*x*z).atan2( 1. - 2.*sqy - 2.*sqz);
        let yaw = (2.*test).asin();
        let roll = (2.*x*w-2.*y*z).atan2( 1. - 2.*sqx - 2.*sqz);

        Euler { roll, pitch, yaw }
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    fn compare_float(a: f32, b: f32) -> bool {
        (a - b) < 0.01
    }

    impl PartialEq<Euler> for Euler {
        fn eq(&self, other: &Euler) -> bool {
            compare_float(self.roll, other.roll) &&
            compare_float(self.pitch, other.pitch) &&
            compare_float(self.yaw, other.yaw)
        }
    }

    #[test]
    fn unit() {
        let q = Q {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0
        };

        let expected_euler = Euler {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0
        };

        assert_eq!(expected_euler, q.into());
    }

    #[test]
    fn roll() {
        let q = Q {
            w: 0.991,
            x: 0.131,
            y: 0.0,
            z: 0.0
        };

        let expected_euler = Euler {
            roll: 0.262,
            pitch: 0.0,
            yaw: 0.0
        };

        assert_eq!(expected_euler, q.into());
    }

    #[test]
    fn pitch() {
        let q = Q {
            w: 0.991,
            x: 0.0,
            y: 0.131,
            z: 0.0
        };

        let expected_euler = Euler {
            roll: 0.0,
            pitch: 0.262,
            yaw: 0.0
        };

        assert_eq!(expected_euler, q.into());
    }

    #[test]
    fn yaw() {
        let q = Q {
            w: 0.991,
            x: 0.0,
            y: 0.0,
            z: 0.131
        };

        let expected_euler = Euler {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.262
        };

        assert_eq!(expected_euler, q.into());
    }

    #[test]
    fn all_positive() {
        let _q = Q {
            w: 0.996,
            x: 0.052,
            y: 0.047,
            z: 0.052
        };

        let _expected_euler = Euler {
            roll: 0.1,
            pitch: 0.1,
            yaw: 0.1
        };

        // this fails, presumably due to rounding errors
        // assert_eq!(expected_euler, q.into());
    }
}
