/// # Madgwick algorithms
/// Implement the two madgwick algorithms detailed here:
/// https://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf
///
/// The two algorithms differ with their use of magnetic data. Both are present here.
///
/// Initial Q for a system is 1,0,0,0

extern crate linux_embedded_hal as hal;
extern crate mpu9250;

use std::thread::sleep;
use std::time::Duration;

use hal::{Delay, Pin, Spidev};
use hal::spidev::{self, SpidevOptions};
use hal::sysfs_gpio::Direction;
use mpu9250::{MargMeasurements, Mpu9250};

use marg::{Q, V};
use marg::madgwick;

use self::mpu9250::{Marg, SpiDevice};

use crate::{Q, V};

// using hardcoded value of pi to match paper
const GYRO_MEAS_ERROR: f32 = std::f32::consts::PI * (5.0 / 180.0);  // gyroscope measurement error in rad/s (shown as 5 deg/s)
const GYRO_MEAS_DRIFT: f32 = std::f32::consts::PI * (0.2 / 180.0);  // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
// Compute beta and zeta in function because of rust no sqrt fn in const.


#[derive(Debug)]
pub struct MargMeta {
    pub b_z: f32,
    pub b_x: f32,
    pub w_bx: f32,
    pub w_by: f32,
    pub w_bz: f32,
}

impl Default for MargMeta {
    fn default() -> Self {
        MargMeta{
        b_z:0.0,
        b_x: 1.0,
        w_bx: 0.0,
        w_by: 0.0,
        w_bz: 0.0,
        }
    }
}

#[allow(non_snake_case)]
pub fn madgwick_marg(w: V, a: V, m: V, q: Q, deltat: f32, meta: MargMeta) -> (Q, MargMeta) {

    let beta: f32 = (3_f32 / 4_f32).sqrt() * GYRO_MEAS_ERROR;
    let zeta: f32 = (3_f32 / 4_f32).sqrt() * GYRO_MEAS_DRIFT;

    let b_z = meta.b_z;
    let b_x = meta.b_x;
    let mut w_bx = meta.w_bx;
    let mut w_by = meta.w_by;
    let mut w_bz = meta.w_bz;

    // Convert w, m, a and q structs to variable names used in the paper.
    let mut SEq_1 = q.w;
    let mut SEq_2 = q.x;
    let mut SEq_3 = q.y;
    let mut SEq_4 = q.z;

    let mut a_x = a.x;
    let mut a_y = a.y;
    let mut a_z = a.z;

    let mut w_x = w.x;
    let mut w_y = w.y;
    let mut w_z = w.z;

    let mut m_x = m.x;
    let mut m_y = m.y;
    let mut m_z = m.z;

    // axulirary variables to avoid reapeated calcualtions
    let halfSEq_1 = 0.5 * SEq_1;
    let halfSEq_2 = 0.5 * SEq_2;
    let halfSEq_3 = 0.5 * SEq_3;
    let halfSEq_4 = 0.5 * SEq_4;
    let twoSEq_1 = 2.0 * SEq_1;
    let twoSEq_2 = 2.0 * SEq_2;
    let twoSEq_3 = 2.0 * SEq_3;
    let twoSEq_4 = 2.0 * SEq_4;
    let twob_x = 2.0 * b_x;
    let twob_z = 2.0 * b_z;
    let twob_xSEq_1 = 2.0 * b_x * SEq_1;
    let twob_xSEq_2 = 2.0 * b_x * SEq_2;
    let twob_xSEq_3 = 2.0 * b_x * SEq_3;
    let twob_xSEq_4 = 2.0 * b_x * SEq_4;
    let twob_zSEq_1 = 2.0 * b_z * SEq_1;
    let twob_zSEq_2 = 2.0 * b_z * SEq_2;
    let twob_zSEq_3 = 2.0 * b_z * SEq_3;
    let twob_zSEq_4 = 2.0 * b_z * SEq_4;
    // let SEq_1SEq_2;
    let SEq_1SEq_3 = SEq_1 * SEq_3;
    // let SEq_1SEq_4;
    // let SEq_2SEq_3;
    let SEq_2SEq_4 = SEq_2 * SEq_4;
    // let SEq_3SEq_4;
    let twom_x = 2.0 * m_x;
    let twom_y = 2.0 * m_y;
    let twom_z = 2.0 * m_z;

    // normalise the accelerometer measurement
    let accel_norm = (a_x * a_x + a_y * a_y + a_z * a_z).sqrt();
    a_x /= accel_norm;
    a_y /= accel_norm;
    a_z /= accel_norm;

    // normalise the magnetometer measurement
    let mag_norm = (m_x * m_x + m_y * m_y + m_z * m_z).sqrt();
    m_x /= mag_norm;
    m_y /= mag_norm;
    m_z /= mag_norm;

    //compute the objective function and Jacobian
    let f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
    let f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
    let f_3 = 1.0 - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
    let f_4 = twob_x * (0.5 - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
    let f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y;
    let f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5 - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z;
    let J_11or24 = twoSEq_3;  // J_11 negated in matrix multiplication
    let J_12or23 = 2.0 * SEq_4;
    let J_13or22 = twoSEq_1;  // J_12 negated in matrix multiplication
    let J_14or21 = twoSEq_2;
    let J_32 = 2.0 * J_14or21;  // negated in matrix multiplication
    let J_33 = 2.0 * J_11or24; // negated in matrix multiplication
    let J_41 = twob_zSEq_3;  // negated in matrix multiplication
    let J_42 = twob_zSEq_4;
    let J_43 = 2.0 * twob_xSEq_3 + twob_zSEq_1;   // negated in matrix multiplication
    let J_44 = 2.0 * twob_xSEq_4 - twob_zSEq_2;   // negated in matrix multiplication
    let J_51 = twob_xSEq_4 - twob_zSEq_2;    // negated in matrix multiplication
    let J_52 = twob_xSEq_3 + twob_zSEq_1;
    let J_53 = twob_xSEq_2 + twob_zSEq_4;
    let J_54 = twob_xSEq_1 - twob_zSEq_3;  // negated in matrix multiplication
    let J_61 = twob_xSEq_3;
    let J_62 = twob_xSEq_4 - 2.0 * twob_zSEq_2;
    let J_63 = twob_xSEq_1 - 2.0 * twob_zSEq_3;
    let J_64 = twob_xSEq_2;

    // compute the gradient (matrix multiplication)
    let mut SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
    let mut SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
    let mut SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
    let mut SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;

    // normalise the gradient to estimate direction of the gyroscope error
    let norm = (SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4).sqrt();
    SEqHatDot_1 = SEqHatDot_1 / norm;
    SEqHatDot_2 = SEqHatDot_2 / norm;
    SEqHatDot_3 = SEqHatDot_3 / norm;
    SEqHatDot_4 = SEqHatDot_4 / norm;

    // compute angular estimated direction of the gyroscope error
    let w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
    let w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
    let w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;

    // compute and remove the gyroscope baises
    w_bx += w_err_x * deltat * zeta;
    w_by += w_err_y * deltat * zeta;
    w_bz += w_err_z * deltat * zeta;
    w_x -= w_bx;
    w_y -= w_by;
    w_z -= w_bz;

    // compute the quaternion rate measured by gyroscopes
    let SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    let SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    let SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    let SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

    // compute then integrate the estimated quaternion rate
    SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
    SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
    SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
    SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;

    // normalise quaternion
    let norm = (SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4).sqrt();
    SEq_1 /= norm;
    SEq_2 /= norm;
    SEq_3 /= norm;
    SEq_4 /= norm;

    // compute flux in the earth frame
    // recompute axulirary variables
    let SEq_1SEq_2 = SEq_1 * SEq_2;
    let SEq_1SEq_3 = SEq_1 * SEq_3;
    let SEq_1SEq_4 = SEq_1 * SEq_4;
    let SEq_3SEq_4 = SEq_3 * SEq_4;
    let SEq_2SEq_3 = SEq_2 * SEq_3;
    let SEq_2SEq_4 = SEq_2 * SEq_4;
    let h_x = twom_x * (0.5 - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
    let h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5 - SEq_2 * SEq_2 - SEq_4 * SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
    let h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5 - SEq_2 * SEq_2 - SEq_3 * SEq_3);

    // normalise the flux vector to have only components in the x and z
    let b_x = ((h_x * h_x) + (h_y * h_y)).sqrt();
    let b_z = h_z;

    let q = Q{
        w: SEq_1,
        x: SEq_2,
        y: SEq_3,
        z: SEq_4,
    };

    let meta = MargMeta{
        b_z,
        b_x,
        w_bx,
        w_by,
        w_bz
    };

    return (q, meta)
}

pub fn madgwick_imu(w: V, mut a: V, mut q: Q, delta_t: f32) -> Q {
    //! Implementation of Madgwick's algorithm
    //!
    //! - w: gyroscope measurements in rad/s, as V struct
    //! - a: accelerometer measurements, as V struct
    //! - q: orientation quaternion elements initial conditions, as Q struct
    //! - delta_t: sampling period in seconds
    //!
    //!
    let beta = (3_f32 / 4_f32).sqrt() * GYRO_MEAS_ERROR;
    let half_seq_1 = 0.5 * q.w;
    let half_seq_2 = 0.5 * q.x;
    let half_seq_3 = 0.5 * q.y;
    let half_seq_4 = 0.5 * q.z;
    let two_seq_1 = 2.0 * q.w;
    let two_seq_2 = 2.0 * q.x;
    let two_seq_3 = 2.0 * q.y;

    let mut norm = (a.x * a.x + a.y * a.y + a.z * a.z).sqrt();
    a.x /= norm;
    a.y /= norm;
    a.z /= norm;

    let f_1 = two_seq_2 * q.z - two_seq_1 * q.y - a.x;
    let f_2 = two_seq_1 * q.x + two_seq_3 * q.z - a.y;
    let f_3 = 1.0 - two_seq_2 * q.x - two_seq_3 * q.y - a.z;
    let j_11_or_24 = two_seq_3;
    let j_12_or_23 = 2.0 * q.z;
    let j_13_or_22 = two_seq_1;
    let j_14_or_21 = two_seq_2;
    let j_32 = 2.0 * j_14_or_21;
    let j_33 = 2.0 * j_11_or_24;

    let mut seq_hat_dot_1 = j_14_or_21 * f_2 - j_11_or_24 * f_1;
    let mut seq_hat_dot_2 = j_12_or_23 * f_1 + j_13_or_22 * f_2 - j_32 * f_3;
    let mut seq_hat_dot_3 = j_12_or_23 * f_2 - j_33 * f_3 - j_13_or_22 * f_1;
    let mut seq_hat_dot_4 = j_14_or_21 * f_1 + j_11_or_24 * f_2;

    norm = (
        seq_hat_dot_1 * seq_hat_dot_1 +
            seq_hat_dot_2 * seq_hat_dot_2 +
            seq_hat_dot_3 * seq_hat_dot_3 +
            seq_hat_dot_4 * seq_hat_dot_4
    ).sqrt();
    seq_hat_dot_1 /= norm;
    seq_hat_dot_2 /= norm;
    seq_hat_dot_3 /= norm;
    seq_hat_dot_4 /= norm;

    let seq_dot_omega_1 = -half_seq_2 * w.x - half_seq_3 * w.y - half_seq_4 * w.z;
    let seq_dot_omega_2 = half_seq_1 * w.x + half_seq_3 * w.z - half_seq_4 * w.y;
    let seq_dot_omega_3 = half_seq_1 * w.y + half_seq_2 * w.z + half_seq_4 * w.x;
    let seq_dot_omega_4 = half_seq_1 * w.z + half_seq_2 * w.y - half_seq_3 * w.x;

    q.w += (seq_dot_omega_1 - (beta * seq_hat_dot_1)) * delta_t;
    q.x += (seq_dot_omega_2 - (beta * seq_hat_dot_2)) * delta_t;
    q.y += (seq_dot_omega_3 - (beta * seq_hat_dot_3)) * delta_t;
    q.z += (seq_dot_omega_4 - (beta * seq_hat_dot_4)) * delta_t;

    norm = (q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z).sqrt();
    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;

    q
}

