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

pub struct Mpu {
    pub port: Mpu9250<SpiDevice<Spidev, Pin>, Marg>
}

pub fn open_mpu_port() -> Mpu {
    let mut spi = Spidev::open("/dev/spidev0.0").unwrap();
    let options = SpidevOptions::new().max_speed_hz(1_000_000)
        .mode(spidev::SPI_MODE_3)
        .build();
    spi.configure(&options).unwrap();

    let ncs = Pin::new(25);
    ncs.export().unwrap();
    sleep(Duration::from_millis(100));  // Seems to fix set_direction permission issue
    while !ncs.is_exported() {}

    ncs.set_direction(Direction::Out).unwrap();  // Permission error here on first run.
    ncs.set_value(1).unwrap();

    let mpu = Mpu9250::marg_default(spi, ncs, &mut Delay).unwrap();
    return Mpu {
        port: mpu,
    };
}

pub fn close_mpu_port() {
    let _ = Pin::new(25).unexport().unwrap();
    sleep(Duration::from_millis(100));
}


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
        let meta = madgwick::MargMeta::default();
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