/// # Magnetometer and accelerometer calibration
/// https://medium.com/@niru5/intro-to-inertial-measurement-unit-imu-part-1-47f19fc7d68d
///
/// ## Magnetometer calibration
/// Reference: https://teslabs.com/articles/magnetometer-calibration/
/// To remove hard iron and soft iron distortion of the magnetometer, there must be calibration.
///
/// ## Accelerometer calibration
/// When at rest, working out the direction of gravity and removing it to calculate acceleration
///
///


pub struct MagOffset{
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

// impl MagOffset{
//     pub fn mag_cal() {
//         //! Data in the x, y, z format.
//         // Read data for x number of seconds
//         let x = [];
//         let y = [];
//         let z = [];
//
//         offset_x = (x.max() + x.min())/ 2;
//         offset_y = (y.max() + y.min())/ 2;
//         offset_z = (z.max() + z.min())/ 2;
//
//
//
//     }
//
// }









pub fn accel_cal() {
    //! Must run when the chip is at rest to get and remove gravity
}




