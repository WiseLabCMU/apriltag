/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
//#include <unistd.h>
#include <math.h>

#include "apriltag.h"
#include "apriltag_pose.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"

#include "common/getopt.h"
#include "common/image_u8.h"
#include "common/image_u8x4.h"
#include "common/pjpeg.h"
#include "common/zarray.h"

#define HAMM_HIST_MAX   (10)

// Invoke:
//
// tagtest [options] input.pnm

// defaults to a 150mm tag and camera intrinsics of a MacBook Pro (13-inch, 2018)
// apriltag_detection_info_t: {apriltag_detection_t*, tagsize_meters, fx, fy, cx, cy}
// NOTE: tag size is assumed according to the tag id:[0,150]=150mm;  ]150,300]=100mm; ]300,450]=50mm; ]450,587]=20mm;
// static apriltag_detection_info_t g_det_pose_info = {NULL, 0.150, 112.5, 112.5, 112.5, 112.5};
static apriltag_detection_info_t g_det_pose_info = {
    NULL,
    0.150,
    997.5703125,
    997.5703125,
    636.8616333007812,
    360.50408935546875 };

static const char fmt_det_point_pose[] = "{\n\t\"id\":%d,\n\t\"size\":%.2f,\n\t\"corners\": [\n\t\t{\"x\":%.2f,\"y\":%.2f},\n\t\t{\"x\":%.2f,\"y\":%.2f},\n\t\t{\"x\":%.2f,\"y\":%.2f},\n\t\t{\"x\":%.2f,\"y\":%.2f}\n\t],\n\t\"center\": {\"x\":%.2f,\"y\":%.2f},\n\t\"pose\": {\n\t\t\"R\": [\n\t\t\t[%f,%f,%f],\n\t\t\t[%f,%f,%f],\n\t\t\t[%f,%f,%f]\n\t\t],\n\t\t\"t\": [%f,%f,%f],\n\t\t\"e\": %f\n\t}\n}\n";

int main(int argc, char *argv[])
{
    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, 'i', "iters", "1", "Repeat processing on input set this many times");
    getopt_add_int(getopt, 't', "threads", "1", "Use this many CPU threads");
    getopt_add_int(getopt, 'a', "hamming", "1", "Detect tags with up to this many bit errors.");
    getopt_add_double(getopt, 'x', "decimate", "2.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input; negative sharpens");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");

    if (!getopt_parse(getopt, argc, argv, 1) || getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options] <input files>\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    const zarray_t *inputs = getopt_get_extra_args(getopt);

    apriltag_family_t *tf = NULL;
    const char *famname = getopt_get_string(getopt, "family");
    if (!strcmp(famname, "tag36h11")) {
        tf = tag36h11_create();
    } else if (!strcmp(famname, "tag25h9")) {
        tf = tag25h9_create();
    } else if (!strcmp(famname, "tag16h5")) {
        tf = tag16h5_create();
    } else if (!strcmp(famname, "tagCircle21h7")) {
        tf = tagCircle21h7_create();
    } else if (!strcmp(famname, "tagCircle49h12")) {
        tf = tagCircle49h12_create();
    } else if (!strcmp(famname, "tagStandard41h12")) {
        tf = tagStandard41h12_create();
    } else if (!strcmp(famname, "tagStandard52h13")) {
        tf = tagStandard52h13_create();
    } else if (!strcmp(famname, "tagCustom48h12")) {
        tf = tagCustom48h12_create();
    } else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family_bits(td, tf, getopt_get_int(getopt, "hamming"));
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");

    int quiet = getopt_get_bool(getopt, "quiet");

    int maxiters = getopt_get_int(getopt, "iters");

    for (int iter = 0; iter < maxiters; iter++) {

        int total_quads = 0;
        int total_hamm_hist[HAMM_HIST_MAX];
        memset(total_hamm_hist, 0, sizeof(total_hamm_hist));
        double total_time = 0;

        if (maxiters > 1)
            printf("iter %d / %d\n", iter + 1, maxiters);

        for (int input = 0; input < zarray_size(inputs); input++) {

            int hamm_hist[HAMM_HIST_MAX];
            memset(hamm_hist, 0, sizeof(hamm_hist));

            char *path;
            zarray_get(inputs, input, &path);
            if (!quiet)
                printf("loading %s\n", path);
            else
                printf("%20s ", path);

            image_u8_t *im = NULL;
            if (str_ends_with(path, "pnm") || str_ends_with(path, "PNM") ||
                str_ends_with(path, "pgm") || str_ends_with(path, "PGM"))
                im = image_u8_create_from_pnm(path);
            else if (str_ends_with(path, "jpg") || str_ends_with(path, "JPG")) {
                int err = 0;
                pjpeg_t *pjpeg = pjpeg_create_from_file(path, 0, &err);
                if (pjpeg == NULL) {
                    printf("pjpeg error %d\n", err);
                    continue;
                }

                if (1) {
                    im = pjpeg_to_u8_baseline(pjpeg);
                } else {
                    printf("illumination invariant\n");

                    image_u8x3_t *imc =  pjpeg_to_u8x3_baseline(pjpeg);

                    im = image_u8_create(imc->width, imc->height);

                    for (int y = 0; y < imc->height; y++) {
                        for (int x = 0; x < imc->width; x++) {
                            double r = imc->buf[y*imc->stride + 3*x + 0] / 255.0;
                            double g = imc->buf[y*imc->stride + 3*x + 1] / 255.0;
                            double b = imc->buf[y*imc->stride + 3*x + 2] / 255.0;

                            double alpha = 0.42;
                            double v = 0.5 + log(g) - alpha*log(b) - (1-alpha)*log(r);
                            int iv = v * 255;
                            if (iv < 0)
                                iv = 0;
                            if (iv > 255)
                                iv = 255;

                            im->buf[y*im->stride + x] = iv;
                        }
                    }
                    image_u8x3_destroy(imc);
                    if (td->debug)
                        image_u8_write_pnm(im, "debug_invariant.pnm");
                }

                pjpeg_destroy(pjpeg);
            }

            if (im == NULL) {
                printf("couldn't load %s\n", path);
                continue;
            }

            zarray_t *detections = apriltag_detector_detect(td, im);

            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);

                if (!quiet)
                    printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
                           i, det->family->nbits, det->family->h, det->id, det->hamming, det->decision_margin);

                apriltag_pose_t pose0;
                apriltag_pose_t pose1;
                apriltag_pose_t pose2;
                double pose_err0 = 0.0;
                double pose_err1;
                double pose_err2;
                g_det_pose_info.det = det;
                // det->id = 2;

                // struct quad quad;
                // quad.p[3][0] = 806.29;
                // quad.p[3][1] = 246.9;
                // quad.p[2][0] = 883.52;
                // quad.p[2][1] = 261.98;
                // quad.p[1][0] = 907.45;
                // quad.p[1][1] = 179.88;
                // quad.p[0][0] = 829.07;
                // quad.p[0][1] = 164.06;
                // quad.reversed_border = false;
                // quad.H = NULL;
                // quad.Hinv = NULL;

                // quad_update_homographies(&quad);
                // double theta = 0 /*entry.rotation*/ * M_PI / 2.0;
                // double c = cos(theta), s = sin(theta);

                // // Fix the rotation of our homography to properly orient the tag
                // matd_t *R = matd_create(3, 3);
                // MATD_EL(R, 0, 0) = c;
                // MATD_EL(R, 0, 1) = -s;
                // MATD_EL(R, 1, 0) = s;
                // MATD_EL(R, 1, 1) = c;
                // MATD_EL(R, 2, 2) = 1;

                // det->H = matd_op("M*M", quad.H, R);

                // matd_destroy(R);

                // homography_project(det->H, 0, 0, &det->c[0], &det->c[1]);

                // // [-1, -1], [1, -1], [1, 1], [-1, 1], Desired points
                // // [-1, 1], [1, 1], [1, -1], [-1, -1], FLIP Y
                // // adjust the points in det->p so that they correspond to
                // // counter-clockwise around the quad, starting at -1,-1.
                // for (int i = 0; i < 4; i++)
                // {
                //     int tcx = (i == 1 || i == 2) ? 1 : -1;
                //     int tcy = (i < 2) ? 1 : -1;

                //     double p[2];

                //     homography_project(det->H, tcx, tcy, &p[0], &p[1]);

                //     det->p[i][0] = p[0];
                //     det->p[i][1] = p[1];
                // }

                estimate_pose_for_tag_homography(&g_det_pose_info, &pose0);
                estimate_tag_pose_orthogonal_iteration(&g_det_pose_info, &pose_err1, &pose1, &pose_err2, &pose2, 50);
                printf("solution0:\n");
                printf(fmt_det_point_pose, det->id, 0.150, det->p[0][0], det->p[0][1], det->p[1][0], det->p[1][1], det->p[2][0], det->p[2][1], det->p[3][0], det->p[3][1], det->c[0], det->c[1], matd_get(pose0.R, 0, 0), matd_get(pose0.R, 0, 1), matd_get(pose0.R, 0, 2), matd_get(pose0.R, 1, 0), matd_get(pose0.R, 1, 1), matd_get(pose0.R, 1, 2), matd_get(pose0.R, 2, 0), matd_get(pose0.R, 2, 1), matd_get(pose0.R, 2, 2), matd_get(pose0.t, 0, 0), matd_get(pose0.t, 1, 0), matd_get(pose0.t, 2, 0), pose_err0);
                printf("solution1:\n");
                printf(fmt_det_point_pose, det->id, 0.150, det->p[0][0], det->p[0][1], det->p[1][0], det->p[1][1], det->p[2][0], det->p[2][1], det->p[3][0], det->p[3][1], det->c[0], det->c[1], matd_get(pose1.R, 0, 0), matd_get(pose1.R, 0, 1), matd_get(pose1.R, 0, 2), matd_get(pose1.R, 1, 0), matd_get(pose1.R, 1, 1), matd_get(pose1.R, 1, 2), matd_get(pose1.R, 2, 0), matd_get(pose1.R, 2, 1), matd_get(pose1.R, 2, 2), matd_get(pose1.t, 0, 0), matd_get(pose1.t, 1, 0), matd_get(pose1.t, 2, 0), pose_err1);
                if (pose2.R)
                {
                    printf("solution2:\n");
                    printf(fmt_det_point_pose, det->id, 0.150, det->p[0][0], det->p[0][1], det->p[1][0], det->p[1][1], det->p[2][0], det->p[2][1], det->p[3][0], det->p[3][1], det->c[0], det->c[1], matd_get(pose2.R, 0, 0), matd_get(pose2.R, 0, 1), matd_get(pose2.R, 0, 2), matd_get(pose2.R, 1, 0), matd_get(pose2.R, 1, 1), matd_get(pose2.R, 1, 2), matd_get(pose2.R, 2, 0), matd_get(pose2.R, 2, 1), matd_get(pose2.R, 2, 2), matd_get(pose2.t, 0, 0), matd_get(pose2.t, 1, 0), matd_get(pose2.t, 2, 0), pose_err2);
                    matd_destroy(pose2.R);
                    matd_destroy(pose2.t);
                }
                matd_destroy(pose1.R);
                matd_destroy(pose1.t);

                hamm_hist[det->hamming]++;
                total_hamm_hist[det->hamming]++;
            }

            apriltag_detections_destroy(detections);

            if (!quiet) {
                timeprofile_display(td->tp);
            }

            total_quads += td->nquads;

            if (!quiet)
                printf("hamm ");

            for (int i = 0; i < HAMM_HIST_MAX; i++)
                printf("%5d ", hamm_hist[i]);

            double t =  timeprofile_total_utime(td->tp) / 1.0E3;
            total_time += t;
            printf("%12.3f ", t);
            printf("%5d", td->nquads);

            printf("\n");

            image_u8_destroy(im);
        }


        printf("Summary\n");

        printf("hamm ");

        for (int i = 0; i < HAMM_HIST_MAX; i++)
            printf("%5d ", total_hamm_hist[i]);
        printf("%12.3f ", total_time);
        printf("%5d", total_quads);
        printf("\n");

    }

    // don't deallocate contents of inputs; those are the argv
    apriltag_detector_destroy(td);

    if (!strcmp(famname, "tag36h11")) {
        tag36h11_destroy(tf);
    } else if (!strcmp(famname, "tag25h9")) {
        tag25h9_destroy(tf);
    } else if (!strcmp(famname, "tag16h5")) {
        tag16h5_destroy(tf);
    } else if (!strcmp(famname, "tagCircle21h7")) {
        tagCircle21h7_destroy(tf);
    } else if (!strcmp(famname, "tagCircle49h12")) {
        tagCircle49h12_destroy(tf);
    } else if (!strcmp(famname, "tagStandard41h12")) {
        tagStandard41h12_destroy(tf);
    } else if (!strcmp(famname, "tagStandard52h13")) {
        tagStandard52h13_destroy(tf);
    } else if (!strcmp(famname, "tagCustom48h12")) {
        tagCustom48h12_destroy(tf);
    }

    getopt_destroy(getopt);

    return 0;
}
