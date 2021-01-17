use anyhow::Result;
use approx::abs_diff_eq;
use cam_geom::{ExtrinsicParameters, IntrinsicParameters, Points};
use cv_convert::{OpenCvPose, TryFromCv};
use nalgebra as na;
use opencv::{calib3d, core};
use opencv_ros_camera::{Distortion, RosOpenCvIntrinsics};
use rand::prelude::*;
use std::f64;

fn main() -> Result<()> {
    let mut rng = rand::thread_rng();

    const MAX_COORD: f64 = 10.0;
    const MAX_TRANSLATION: f64 = 0.1;
    const MAX_ANGLEL: f64 = f64::consts::PI * 2.0;
    const MIN_ANGLEL: f64 = 0.0;
    const MAX_F: f64 = 2.0;
    const MIN_F: f64 = 0.1;
    const MAX_C: f64 = 1000.0;
    const MIN_C: f64 = -1000.0;
    const MAX_D: f64 = 0.5;
    const MIN_D: f64 = -0.5;
    const N_POINTS: usize = 1;
    const N_ITERATIONS: usize = 10000;
    const EPSILON: f64 = 1e-3;

    let mut n_fails = 0;

    for _ in 0..N_ITERATIONS {
        // sample pose
        let pose = {
            let translation = {
                let x = rng.gen_range(-MAX_TRANSLATION..MAX_TRANSLATION);
                let y = rng.gen_range(-MAX_TRANSLATION..MAX_TRANSLATION);
                let z = rng.gen_range(-MAX_TRANSLATION..MAX_TRANSLATION);
                na::Translation3::new(x, y, z)
            };
            let rotation = {
                let roll = rng.gen_range(MIN_ANGLEL..MAX_ANGLEL);
                let pitch = rng.gen_range(MIN_ANGLEL..MAX_ANGLEL);
                let yaw = rng.gen_range(MIN_ANGLEL..MAX_ANGLEL);
                na::UnitQuaternion::from_euler_angles(roll, pitch, yaw)
            };

            na::Isometry3 {
                translation,
                rotation,
            }
        };

        // create extrinsic params
        let ext_params = ExtrinsicParameters::from_pose(&pose);

        // sample intrinsic params
        let (camera_matrix, distortoin_vec, int_params) = {
            let fx = rng.gen_range(MIN_F..MAX_F);
            let fy = rng.gen_range(MIN_F..MAX_F);

            let skew = 0.0;

            let cx = rng.gen_range(MIN_C..MAX_C);
            let cy = rng.gen_range(MIN_C..MAX_C);

            let k1 = rng.gen_range(MIN_D..MAX_D);
            let k2 = rng.gen_range(MIN_D..MAX_D);
            let k3 = rng.gen_range(MIN_D..MAX_D);
            let p1 = rng.gen_range(MIN_D..MAX_D);
            let p2 = rng.gen_range(MIN_D..MAX_D);

            let distortion_vec = na::Vector5::new(k1, k2, p1, p2, k3);
            let camera_matrix = na::Matrix3::new(
                fx, 0.0, cx, // row 1
                0.0, fy, cy, // row 2
                0.0, 0.0, 1.0, // row 3
            );
            let int_params = RosOpenCvIntrinsics::from_params_with_distortion(
                fx,
                skew,
                fy,
                cx,
                cy,
                Distortion::from_opencv_vec(distortion_vec.clone()),
            );

            (camera_matrix, distortion_vec, int_params)
        };

        // sample points
        let points = Points::new(na::MatrixXx3::from_iterator(
            N_POINTS,
            (0..(3 * N_POINTS)).map(|_| rng.gen_range(-MAX_COORD..MAX_COORD)),
        ));

        // compute from rust
        let rust_pixels = int_params
            .camera_to_pixel(&ext_params.world_to_camera(&points))
            .data;

        // compute from opencv
        let opencv_pixels = {
            let OpenCvPose { rvec, tvec } = OpenCvPose::<core::Mat>::try_from_cv(&pose)?;

            let object_points = core::Mat::try_from_cv(&points.data)?;
            let mut image_points: core::Vector<core::Point2d> = core::Vector::new();
            let cv_camera_matrix = core::Mat::try_from_cv(&camera_matrix)?;
            let dist_coefs = core::Mat::try_from_cv(&distortoin_vec)?;
            let mut jacobian = core::no_array()?;
            let aspect_ratio = 0.0;

            calib3d::project_points(
                &object_points,
                &rvec,
                &tvec,
                &cv_camera_matrix,
                &dist_coefs,
                &mut image_points,
                &mut jacobian,
                aspect_ratio,
            )?;

            let n_points = image_points.len();
            na::MatrixXx2::from_fn(n_points, |row, col| {
                let point = &image_points.as_slice()[row];
                match col {
                    0 => point.x,
                    1 => point.y,
                    _ => unreachable!(),
                }
            })
        };

        let (roll, pitch, yaw) = ext_params.rotation().euler_angles();

        // compute loss

        // check identity
        if !abs_diff_eq!(rust_pixels, opencv_pixels, epsilon = EPSILON) {
            n_fails += 1;

            println!(
                "==== Numerical difference detected ====
extrinsic = {}
(roll, pitch, yaw) = {:?}
translation = {}
(fx, fy, cx, cy) = {:?}
distortion = {}
world_points =  {}
camera_points =  {}
rust_pixels =  {}
opencv_pixels = {}",
                ext_params.matrix(),
                (roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees()),
                ext_params.translation(),
                (
                    int_params.k[(0, 0)],
                    int_params.k[(1, 1)],
                    int_params.k[(0, 2)],
                    int_params.k[(1, 2)],
                ),
                distortoin_vec,
                points.data,
                ext_params.world_to_camera(&points).data,
                rust_pixels,
                opencv_pixels
            );
        }
    }

    println!("use epsilon {}", EPSILON);
    println!("{} test cases", N_ITERATIONS);
    println!("{} pass", N_ITERATIONS - n_fails);
    println!("{} fail", n_fails);
    println!(
        "{:3.3}% success rate",
        (1.0 - n_fails as f64 / N_ITERATIONS as f64) * 100.0
    );

    Ok(())
}
