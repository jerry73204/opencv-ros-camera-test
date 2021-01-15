# opencv-ros-camera-test

The crate tests the numerical results of [opencv-ros-camera](https://github.com/strawlab/opencv-ros-camera) crate against OpenCV C++ implementation.

## Usage

It is executed with an optional `--release` enables release mode.

```sh
cargo run [--release]
```

The program randomly sample extrinsic, intrinsic and distortion parameters, project the world points and compare the pixel coordinates of the two implementations.

It generates the report like this.

```
==== Numerical difference detected ====
(roll, pitch, yaw) = (-132.14016926170137, -20.822204729430325, -81.0265928210432)
translation = {0.09291752669371138, -0.0803834883648712, 0.09773221601803085}
distortion =
  ┌                       ┐
  │   -0.4868987163878924 │
  │   0.11490528936791966 │
  │ -0.004733403121498725 │
  │  -0.44697145318540654 │
  │   -0.4494321323411492 │
  └                       ┘


points =
  ┌                                                          ┐
  │ 1.8415970534513626 -6.126268527271055  8.269151033921073 │
  └                                                          ┘


rust_pixels =
  ┌                                         ┐
  │  1327680462364.3928 -12479750952.388031 │
  └                                         ┘


opencv_pixels =
  ┌                                       ┐
  │ 1327680462364.3018 -12479750952.38636 │
  └                                       ┘


use epsilon 0.001
10000 test cases
9713 pass
287 fail
97.130% success rate
```

## License

MIT. See [LICENSE](LICENSE.txt) file.

