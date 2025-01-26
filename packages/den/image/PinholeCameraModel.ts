// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

import type { CameraInfo } from "./CameraInfo";

type Vector2 = { x: number; y: number };

type Vector3 = { x: number; y: number; z: number };

type Matrix3 = [number, number, number, number, number, number, number, number, number];

// prettier-ignore
type Matrix3x4 = [
  number, number, number, number,
  number, number, number, number,
  number, number, number, number,
];

type Vec8 = [number, number, number, number, number, number, number, number];

/**
 * An interface matching OpenCV-style termination criteria:
 * - type: combination of MAX_ITER (1) and/or EPS (2).
 * - maxCount: maximum iterations if (type & MAX_ITER).
 * - epsilon: minimum step size if (type & EPS).
 */
interface TermCriteria {
  type?: number; // e.g. 1=MAX_ITER, 2=EPS, 3=MAX_ITER|EPS
  maxCount?: number; // e.g. 100
  epsilon?: number; // e.g. 1e-12
}

/** Constants for clarity. */
const TermCriteria_MAX_ITER = 1; // 0x01
const TermCriteria_EPS = 2; // 0x02

/**
 * A pinhole camera model that can be used to rectify, unrectify, and project pixel coordinates.
 * Based on `ROSPinholeCameraModel` from the ROS `image_geometry` package. See
 * <http://docs.ros.org/diamondback/api/image_geometry/html/c++/pinhole__camera__model_8cpp_source.html>
 *
 * See also <http://wiki.ros.org/image_pipeline/CameraInfo>
 */
export class PinholeCameraModel {
  /**
   * Distortion parameters `[k1, k2, p1, p2, k3, k4, k5, k6]`. For `rational_polynomial`, all eight
   * parameters are set. For `plumb_bob`, the last three parameters are set to zero. For no
   * distortion model, all eight parameters are set to zero.
   */
  public D: Readonly<Vec8>;
  /**
   * Intrinsic camera matrix for the raw (distorted) images. 3x3 row-major matrix.
   * ```
   *     [fx  0 cx]
   * K = [ 0 fy cy]
   *     [ 0  0  1]
   * ```
   * Projects 3D points in the camera coordinate frame to 2D pixel coordinates using the focal
   * lengths `(fx, fy)` and principal point `(cx, cy)`.
   */
  public K: Readonly<Matrix3>;
  /**
   * Projection/camera matrix. 3x4 row-major matrix.
   * This matrix specifies the intrinsic (camera) matrix of the processed (rectified) image. That
   * is, the left 3x3 portion is the normal camera intrinsic matrix for the rectified image.
   *
   * It projects 3D points in the camera coordinate frame to 2D pixel coordinates using the focal
   * lengths `(fx', fy')` and principal point `(cx', cy')` - these may differ from the values in K.
   * For monocular cameras, `Tx = Ty = 0`. Normally, monocular cameras will also have R = the
   * identity and `P[1:3,1:3] = K`.
   *
   * For a stereo pair, the fourth column `[Tx Ty 0]'` is related to the position of the optical
   * center of the second camera in the first camera's frame. We assume `Tz = 0` so both cameras are
   * in the same stereo image plane. The first camera always has `Tx = Ty = 0`. For the right
   * (second) camera of a horizontal stereo pair, `Ty = 0 and Tx = -fx' * B`, where `B` is the
   * baseline between the cameras.
   *
   * Given a 3D point `[X Y Z]'`, the projection `(x, y)` of the point onto the rectified image is
   * given by:
   * ```
   * [u v w]' = P * [X Y Z 1]'
   *        x = u / w
   *        y = v / w
   * ```
   * This holds for both images of a stereo pair.
   */
  public P: Readonly<Matrix3x4>;
  /**
   * Rectification matrix (stereo cameras only). 3x3 row-major matrix.
   * A rotation matrix aligning the camera coordinate system to the ideal stereo image plane so
   * that epipolar lines in both stereo images are parallel.
   */
  public R: Readonly<Matrix3>;
  /** The full camera image width in pixels. */
  public readonly width: number;
  /** The full camera image height in pixels. */
  public readonly height: number;
  readonly #model: string;

  // Mostly copied from `fromCameraInfo`
  // <http://docs.ros.org/diamondback/api/image_geometry/html/c++/pinhole__camera__model_8cpp_source.html#l00064>
  public constructor(info: CameraInfo) {
    const { binning_x, binning_y, roi, distortion_model: model, D, K, P, R, width, height } = info;
    const fx = P[0];
    const fy = P[5];

    if (width <= 0 || height <= 0) {
      throw new Error(`Invalid image size ${width}x${height}`);
    }
    if (
      model.length > 0 &&
      model !== "plumb_bob" &&
      model !== "rational_polynomial" &&
      model !== "fisheye" &&
      model !== "equidistant"
    ) {
      throw new Error(`Unrecognized distortion_model "${model}"`);
    }
    if (K.length !== 0 && K.length !== 9) {
      throw new Error(`K.length=${K.length}, expected 9`);
    }
    if (P.length !== 12) {
      throw new Error(`P.length=${P.length}, expected 12`);
    }
    if (R.length !== 0 && R.length !== 9) {
      throw new Error(`R.length=${R.length}, expected 9`);
    }
    if (fx === 0 || fy === 0) {
      throw new Error(`Invalid focal length (fx=${fx}, fy=${fy})`);
    }

    const D8 = [...D];
    while (D8.length < 8) {
      D8.push(0);
    }
    this.D = D8 as Vec8;
    this.K = K.length === 9 ? (K as Matrix3) : [1, 0, 0, 0, 1, 0, 0, 0, 1];
    this.P = P as Matrix3x4;
    this.R = R.length === 9 ? (R as Matrix3) : [1, 0, 0, 0, 1, 0, 0, 0, 1];
    this.width = width;
    this.height = height;
    this.#model = model;

    // Binning = 0 is considered the same as binning = 1 (no binning).
    const binningX = binning_x !== 0 ? binning_x : 1;
    const binningY = binning_y !== 0 ? binning_y : 1;

    const adjustBinning = binningX > 1 || binningY > 1;
    const adjustRoi = roi.x_offset !== 0 || roi.y_offset !== 0;

    if (adjustBinning || adjustRoi) {
      throw new Error(
        "Failed to initialize camera model: unable to handle adjusted binning and adjusted roi camera models.",
      );
    }
  }

  /**
   * Undoes camera distortion to map a given coordinate from normalized raw image coordinates to
   * normalized undistorted coordinates.
   *
   * This method uses an iterative optimization algorithm to undo the distortion that was applied to
   * the original image and yields an approximation of the undistorted point.
   *
   * @param out - The output vector to receive the undistorted 2D normalized coordinate.
   * @param point - The input distorted 2D normalized coordinate.
   * @param iterations - The number of iterations to use in the iterative optimization.
   * @returns The undistorted pixel, a reference to `out`.
   */
  public undistortNormalized(out: Vector2, point: Readonly<Vector2>, iterations = 5): Vector2 {
    // Distortion array: [k1, k2, p1, p2, k3, k4, k5, k6]
    const [k1, k2, p1, p2, k3, k4, k5, k6] = this.D;

    // Default TermCriteria if not provided
    const criteria: TermCriteria = {};
    let { type, maxCount, epsilon } = criteria;
    if (type === undefined) {
      // default to MAX_ITER | EPS
      type = TermCriteria_MAX_ITER | TermCriteria_EPS;
    }
    if (maxCount === undefined) {
      maxCount = 20; // default
    }
    if (epsilon === undefined) {
      epsilon = 1e-6; // default
    }

    // We'll use two booleans for clarity:
    // const useMaxIter = (type & TermCriteria_MAX_ITER) !== 0;
    // const useEps = (type & TermCriteria_EPS) !== 0;

    if (this.#model === "fisheye" || this.#model === "equidistant") {
      //
      // ======= FISHEYE/EQUIDISTANT MODEL =======
      //
      // For fish-eye, the first 4 entries of D are [k0, k1, k2, k3].
      // In your array, that means:
      //   k0 = D[0] = k1
      //   k1 = D[1] = k2
      //   k2 = D[2] = p1
      //   k3 = D[3] = p2
      // out.x = point.x;
      // out.y = point.y;
      // return out;
      // const k_0 = k1;
      // const k_1 = k2;
      // const k_2 = p1;
      // const k_3 = p2;

      // Start from the input normalized coords:
      const x0 = point.x;
      const y0 = point.y;
      const r_d = Math.sqrt(x0 * x0 + y0 * y0);

      // If r_d is near zero, no distortion
      if (r_d < 1e-15) {
        out.x = x0;
        out.y = y0;
        return out;
      }

      // return this.#undistortFisheye(
      //   out,
      //   point,
      //   r_d,
      //   [k_0, k_1, k_2, k_3],
      //   maxCount,
      //   useEps,
      //   epsilon,
      // );
      return this.#undistortEquidistant(out, point, r_d, [k1, k2, k3, k4], maxCount);
    }

    // Original radial-tangential distortion code
    // The distortion model is non-linear, so we use fixed-point iteration to
    // incrementally iterate to an approximation of the solution. This approach
    // is described at <http://peterabeles.com/blog/?p=73>. The Jacobi method is
    // used here, balancing accuracy and speed. A more precise method such as
    // Levenberg-Marquardt or the exact formula described in
    // <https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4934233/> could be used,
    // but they are slower and less suitable for real-time applications such as
    // visualization. Note that our method is only locally convergent, requiring
    // a good "initial guess". This means we may not converge for extreme values
    // such as points close to the focal plane.
    //
    // The implementation is based on code from
    // initUndistortRectifyMap and undistortPoints from OpenCV.
    // You can read more about the equations used in the pinhole camera model at
    // <https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#details>

    // See also https://github.com/opencv/opencv/blob/192099352577d18b46840cdaf3cbf365e4c6e663/modules/calib3d/src/undistort.dispatch.cpp

    let x = point.x;
    let y = point.y;
    const x0 = x;
    const y0 = y;
    const count = k1 !== 0 || k2 !== 0 || p1 !== 0 || p2 !== 0 || k3 !== 0 ? iterations : 1;
    for (let i = 0; i < count; ++i) {
      const xx = x * x;
      const yy = y * y;
      const xy = x * y;
      const r2 = xx + yy;
      const r4 = r2 * r2;
      const r6 = r4 * r2;

      const cdist = 1 + k1 * r2 + k2 * r4 + k3 * r6;
      const icdist = (1 + k4 * r2 + k5 * r4 + k6 * r6) / cdist;
      const deltaX = 2 * p1 * xy + p2 * (r2 + 2 * xx);
      const deltaY = p1 * (r2 + 2 * yy) + 2 * p2 * xy;
      x = (x0 - deltaX) * icdist;
      y = (y0 - deltaY) * icdist;
    }

    out.x = x;
    out.y = y;
    return out;
  }

  /**
   * Applies camera distortion parameters to map a given pixel coordinate from normalized
   * undistorted image coordinates to normalized raw coordinates.
   *
   * @param out - The output vector to receive the distorted 2D normalized coordinate
   * @param point - The input undistorted 2D normalized coordinate
   * @returns The distorted pixel, a reference to `out`
   */
  public distortNormalized(out: Vector2, point: Readonly<Vector2>): Vector2 {
    const { R, D } = this;
    const [k1, k2, p1, p2, k3, k4, k5, k6] = D;

    if (this.#model === "fisheye" || this.#model === "equidistant") {
      // for now we just return the point as is
      out.x = point.x;
      out.y = point.y;
      return out;
    }

    // Original radial-tangential distortion code
    // Formulae from:
    // <https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html>

    const x1 = point.x;
    const y1 = point.y;

    // [X Y W]^T <- R^-1 * [x y 1]^T
    const X = R[0] * x1 + R[3] * y1 + R[6];
    const Y = R[1] * x1 + R[4] * y1 + R[7];
    const W = R[2] * x1 + R[5] * y1 + R[8];
    const xp = X / W;
    const yp = Y / W;

    // x'' <- x'(1+k1*r^2+k2*r^4+k3*r^6) / (1 + k4_ * r2 + k5_ * r4 + k6_ * r6) + 2p1*x'*y' + p2(r^2+2x'^2)
    // y'' <- y'(1+k1*r^2+k2*r^4+k3*r^6) / (1 + k4_ * r2 + k5_ * r4 + k6_ * r6) + p1(r^2+2y'^2) + 2p2*x'*y'
    // where r^2 = x'^2 + y'^2

    const xx = xp * xp;
    const yy = yp * yp;
    const xy = xp * yp;
    const r2 = xx + yy;
    const r4 = r2 * r2;
    const r6 = r4 * r2;

    const cdist = (1 + k1 * r2 + k2 * r4 + k3 * r6) / (1 + k4 * r2 + k5 * r4 + k6 * r6);
    const deltaX = 2 * p1 * xy + p2 * (r2 + 2 * xx);
    const deltaY = 2 * p2 * xy + p1 * (r2 + 2 * yy);
    out.x = xp * cdist + deltaX;
    out.y = yp * cdist + deltaY;

    return out;
  }

  /**
   * Undoes camera distortion to map a given pixel coordinate from a raw image to an undistorted image.
   * Similar to OpenCV `undistortPoints()`.
   *
   * @param out - The output undistorted 2D pixel coordinate.
   * @param point - The input distorted 2D pixel coordinate.
   * @param iterations - The number of iterations to use in the iterative optimization.
   * @returns The undistorted pixel, a reference to `out`.
   */
  public undistortPixel(out: Vector2, point: Readonly<Vector2>, iterations = 5): Vector2 {
    const { K, P } = this;
    const fx = K[0];
    const fy = K[4];
    const cx = K[2];
    const cy = K[5];
    const fpx = P[0];
    const fpy = P[5];
    const cpx = P[2];
    const cpy = P[6];

    // Undo K to get normalized coordinates
    out.x = (point.x - cx) / fx;
    out.y = (point.y - cy) / fy;

    // Undo distortion
    this.undistortNormalized(out, out, iterations);

    // Apply K' to get pixel coordinates in the rectified image
    out.x = out.x * fpx + cpx;
    out.y = out.y * fpy + cpy;
    return out;
  }

  /**
   * Applies camera distortion parameters to a given 2D pixel coordinate on an undistorted image,
   * returning the corresponding pixel coordinate on the raw (distorted) image.
   *
   * @param out - The output 2D pixel coordinate on the original (distorted) image
   * @param point - The input 2D pixel coordinate on an undistorted image
   * @returns The distorted pixel, a reference to `out`
   */
  public distortPixel(out: Vector2, point: Readonly<Vector2>): Vector2 {
    out.x = point.x;
    out.y = point.y;

    const { P, K } = this;
    const fx = K[0];
    const fy = K[4];
    const cx = K[2];
    const cy = K[5];

    const fxp = P[0];
    const fyp = P[5];
    const cxp = P[2];
    const cyp = P[6];

    // x <- (u - c'x) / f'x
    // y <- (v - c'y) / f'y
    // c'x, f'x, etc. (primed) come from "new camera matrix" P[0:3, 0:3].
    out.x = (point.x - cxp) / fxp;
    out.y = (point.y - cyp) / fyp;

    this.distortNormalized(out, out);

    // map_x(u,v) <- x''fx + cx
    // map_y(u,v) <- y''fy + cy
    // cx, fx, etc. come from original camera matrix K.
    out.x = out.x * fx + cx;
    out.y = out.y * fy + cy;
    return out;
  }

  /**
   * Projects a 2D image pixel to a point on a plane in 3D world coordinates a
   * unit distance along the Z axis. This is equivalent to `projectPixelTo3dRay`
   * before normalizing.
   *
   * @param out - The output vector to receive the 3D point.
   * @param pixel - The 2D image pixel coordinate.
   * @returns `true` if the projection was successful, or `false` if the camera
   *   projection matrix `P` is not set.
   */
  public projectPixelTo3dPlane(out: Vector3, pixel: Readonly<Vector2>): Vector3 {
    const { K } = this;
    const fx = K[0];
    const fy = K[4];
    const cx = K[2];
    const cy = K[5];

    // Undo K to get normalized coordinates
    out.x = (pixel.x - cx) / fx;
    out.y = (pixel.y - cy) / fy;
    this.undistortNormalized(out, out);

    if (this.#model === "fisheye" || this.#model === "equidistant") {
      // For fisheye, we need to account for the spherical projection
      const r = Math.sqrt(out.x * out.x + out.y * out.y);
      if (r > 0) {
        const theta = Math.atan(r);
        const s = Math.sin(theta) / r;
        out.x *= s;
        out.y *= s;
        out.z = Math.cos(theta);
      } else {
        out.x = 0;
        out.y = 0;
        out.z = 1.0;
      }
    } else {
      out.z = 1.0;
    }
    return out;
  }

  /**
   * Projects a 2D image pixel into a 3D ray in world coordinates. This is
   * equivalent to normalizing the result of `projectPixelTo3dPlane` to get a
   * direction vector.
   *
   * @param out - The output vector to receive the 3D ray direction.
   * @param pixel - The 2D image pixel coordinate.
   * @returns `true` if the projection was successful, or `false` if the camera
   *   projection matrix `P` is not set.
   */
  public projectPixelTo3dRay(out: Vector3, pixel: Readonly<Vector2>): Vector3 {
    this.projectPixelTo3dPlane(out, pixel);

    // Normalize the ray direction
    const invNorm = 1.0 / Math.sqrt(out.x * out.x + out.y * out.y + out.z * out.z);
    out.x *= invNorm;
    out.y *= invNorm;
    out.z *= invNorm;

    return out;
  }

  #undistortFisheye(
    out: Vector2,
    point: Vector2,
    r_d: number,
    K: [number, number, number, number], // [k0, k1, k2, k3]
    maxCount: number,
    useEps: boolean,
    epsilon: number,
  ): Vector2 {
    const [k0, k1_, k2_, k3_] = K;
    const halfPi = Math.PI / 2;
    const theta_d = Math.min(Math.max(r_d, -halfPi), halfPi);

    let theta = theta_d;
    let converged = false;
    let iteration = 0;
    for (; iteration < maxCount; iteration++) {
      const t2 = theta * theta;
      const t4 = t2 * t2;
      const t6 = t4 * t2;
      const t8 = t6 * t2;

      const f0 = theta * (1 + k0 * t2 + k1_ * t4 + k2_ * t6 + k3_ * t8) - theta_d;
      const f0Deriv = 1 + 3 * k0 * t2 + 5 * k1_ * t4 + 7 * k2_ * t6 + 9 * k3_ * t8;

      const step = f0 / f0Deriv;
      theta -= step;

      if (useEps && Math.abs(step) < epsilon) {
        converged = true;
        break;
      }
    }

    const scale = Math.tan(theta) / r_d;

    // Check sign flip, as in C++ code. If sign flips, result is invalid
    const thetaFlipped = (theta_d < 0 && theta > 0) || (theta_d > 0 && theta < 0);

    if ((!useEps || converged) && !thetaFlipped) {
      out.x = point.x * scale;
      out.y = point.y * scale;
    } else {
      out.x = point.x;
      out.y = point.y;
    }
    return out;
  }
  #undistortEquidistant(
    out: Vector2,
    point: Vector2,
    r_d: number,
    K: [number, number, number, number], // [k0, k1, k2, k3]
    maxCount: number,
  ): Vector2 {
    const [k1, k2, k3, k4] = K;
    // For equidistant model:
    // r_d = theta * (1 + k1*theta^2 + k2*theta^4 + k3*theta^6 + k4*theta^8)
    // where theta is the angle from the optical axis
    // We need to solve for theta given r_d

    // Initial guess: theta = r_d
    let theta = r_d;
    // const maxIter = 10;
    const eps = 1e-9;

    // Newton-Raphson iteration to find theta
    for (let i = 0; i < maxCount; i++) {
      const theta2 = theta * theta;
      const theta4 = theta2 * theta2;
      const theta6 = theta4 * theta2;
      const theta8 = theta4 * theta4;

      // f = theta * (1 + k1*theta^2 + k2*theta^4 + k3*theta^6 + k4*theta^8) - r_d
      const f = theta * (1 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8) - r_d;

      // f' = (1 + 3*k1*theta^2 + 5*k2*theta^4 + 7*k3*theta^6 + 9*k4*theta^8)
      const fPrime = 1 + 3 * k1 * theta2 + 5 * k2 * theta4 + 7 * k3 * theta6 + 9 * k4 * theta8;

      const delta = f / fPrime;
      theta -= delta;

      if (Math.abs(delta) < eps) {
        break;
      }
    }

    // Now we have theta, compute the undistorted coordinates
    if (r_d > 0) {
      const scale = Math.tan(theta) / r_d;
      out.x = point.x * scale;
      out.y = point.y * scale;
    } else {
      out.x = point.x;
      out.y = point.y;
    }
    return out;
  }
}
