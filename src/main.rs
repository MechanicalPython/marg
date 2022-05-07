use marg::{open_mpu_port, V, Q, madgwick};
use mpu9250::{MargMeasurements, Mpu9250};


fn main() {
    let mut mpu = open_mpu_port();
    loop {
        let all: MargMeasurements<[f32; 3]> = mpu.port.all().unwrap();
        let accel = all.accel;
        let mag = all.mag;
        let gyro = all.gyro;

        let a = V {
            x: accel[0],
            y: accel[1],
            z: accel[2],
        };
        let _m = V {
            x: mag[0],
            y: mag[1],
            z: mag[2],
        };
        let w = V {
            x: gyro[0],
            y: gyro[1],
            z: gyro[2],
        };
        let _meta = madgwick::MargMeta::default();
        let q = Q::default();

        let q_1 = madgwick::madgwick_imu(w, a, q, 1.0);
        println!("{:?}", q_1.roll_pitch_yaw());
    }

    // let rtm = q_1.0.to_rotation_matrix();
    // println!("{:?}", rtm);

    // let gravity = [[0.0],[0.0],[1.0]];
    // let vector = rtm.dot(&gravity);
    //
    // let accel_true = (rtm.inv().unwrap()).dot(&vector);
    // println!("{:?}", accel_true);
}