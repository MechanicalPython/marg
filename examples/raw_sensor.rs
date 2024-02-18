use marg::{open_mpu_port, V};
use mpu9250::{MargMeasurements};


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
        let m = V {
            x: mag[0],
            y: mag[1],
            z: mag[2],
        };
        let w = V {
            x: gyro[0],
            y: gyro[1],
            z: gyro[2],
        };
        println!("{:?} {:?} {:?}", a, m, w)
    }
}