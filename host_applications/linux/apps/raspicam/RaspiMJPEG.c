/*
Copyright (c) 2013, Broadcom Europe Ltd
Copyright (c) 2013, Silvan Melchior
Copyright (c) 2013, James Hughes
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * \file RaspiMJPEG.c
 * Command line program to capture a camera video stream and encode it to file.
 * Also optionally stream a preview of current camera input wth MJPEG.
 *
 * \date 25th Nov 2013
 * \Author: Silvan Melchior
 *
 * Description
 *
 * RaspiMJPEG is an OpenMAX-Application based on the mmal-library, which is
 * comparable to and inspired by RaspiVid and RaspiStill. RaspiMJPEG can record
 * 1080p 30fps videos and 5 Mpx images into a file. But instead of showing the
 * preview on a screen, RaspiMJPEG streams the preview as MJPEG into a file.
 * The update-rate and the size of the preview are customizable with parameters
 * and independent of the image/video. Once started, the application receives
 * commands with a unix-pipe and showes its status on stdout and writes it into
 * a status-file. The program terminates itself after receiving a SIGINT or
 * SIGTERM.
 *
 * Usage information in README_RaspiMJPEG.md
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <semaphore.h>
#include <signal.h>
#include <fcntl.h>

#include "bcm_host.h"
#include "interface/vcos/vcos.h"
#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

FILE *jpegoutput_file = NULL, *jpegoutput2_file = NULL, *h264output_file = NULL, *status_file = NULL;
MMAL_POOL_T *pool_jpegencoder, *pool_jpegencoder2, *pool_h264encoder;
unsigned int mjpeg_cnt=0, width=320, height=180, divider=5, running=1, quality=85, image_cnt = 0, image2_cnt = 0;
char *jpeg_filename = 0, *jpeg2_filename = 0, *h264_filename = 0, *pipe_filename = 0, *status_filename = 0;


void error (const char *string) {

  fprintf(stderr, "Error: %s\n", string);
  if(status_filename != 0) {
    status_file = fopen(status_filename, "w");
    if(status_file) {
      fprintf(status_file, "error");
      fclose(status_file);
    }
  }
  exit(1);

}

void term (int signum) {

  running = 0;

}

static void camera_control_callback (MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {

  if(buffer->cmd != MMAL_EVENT_PARAMETER_CHANGED) error("Camera sent invalid data");
  mmal_buffer_header_release(buffer);

}

static void jpegencoder_buffer_callback (MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {

  int bytes_written = buffer->length;
  char *filename_temp, *filename_temp2;

  if(mjpeg_cnt == 0) {
    if(!jpegoutput_file) {
      asprintf(&filename_temp, jpeg_filename, image_cnt);
      asprintf(&filename_temp2, "%s.part", filename_temp);
      jpegoutput_file = fopen(filename_temp2, "wb");
      if(!jpegoutput_file) error("Could not open mjpeg-destination");
    }
    if(buffer->length) {
      mmal_buffer_header_mem_lock(buffer);
      bytes_written = fwrite(buffer->data, 1, buffer->length, jpegoutput_file);
      mmal_buffer_header_mem_unlock(buffer);
    }
    if(bytes_written != buffer->length) error("Could not write all bytes");
  }
  
  if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END) {
    mjpeg_cnt++;
    if(mjpeg_cnt == divider) {
      fclose(jpegoutput_file);
      jpegoutput_file = NULL;
      asprintf(&filename_temp, jpeg_filename, image_cnt);
      asprintf(&filename_temp2, "%s.part", filename_temp);
      rename(filename_temp2, filename_temp);
      image_cnt++;
      mjpeg_cnt = 0;
    }
  }

  mmal_buffer_header_release(buffer);

  if (port->is_enabled) {
    MMAL_STATUS_T status = MMAL_SUCCESS;
    MMAL_BUFFER_HEADER_T *new_buffer;

    new_buffer = mmal_queue_get(pool_jpegencoder->queue);

    if (new_buffer) status = mmal_port_send_buffer(port, new_buffer);
    if (!new_buffer || status != MMAL_SUCCESS) error("Could not send buffers to port");
  }

}

static void jpegencoder2_buffer_callback (MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {

  int bytes_written = buffer->length;

  if(buffer->length) {
    mmal_buffer_header_mem_lock(buffer);
    bytes_written = fwrite(buffer->data, 1, buffer->length, jpegoutput2_file);
    mmal_buffer_header_mem_unlock(buffer);
  }
  if(bytes_written != buffer->length) error("Could not write all bytes");
  
  if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END) {
    fclose(jpegoutput2_file);
    if(status_filename != 0) {
      status_file = fopen(status_filename, "w");
      fprintf(status_file, "ready");
      fclose(status_file);
    }
    image2_cnt++;
  }

  mmal_buffer_header_release(buffer);

  if (port->is_enabled) {
    MMAL_STATUS_T status = MMAL_SUCCESS;
    MMAL_BUFFER_HEADER_T *new_buffer;

    new_buffer = mmal_queue_get(pool_jpegencoder2->queue);

    if (new_buffer) status = mmal_port_send_buffer(port, new_buffer);
    if (!new_buffer || status != MMAL_SUCCESS) error("Could not send buffers to port");
  }

}

static void h264encoder_buffer_callback (MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)  {

  int bytes_written = buffer->length;

  if(buffer->length) {
    mmal_buffer_header_mem_lock(buffer);
    bytes_written = fwrite(buffer->data, 1, buffer->length, h264output_file);
    mmal_buffer_header_mem_unlock(buffer);
    if(bytes_written != buffer->length) error("Could not write all bytes");
  }

  mmal_buffer_header_release(buffer);

  if (port->is_enabled) {
    MMAL_STATUS_T status = MMAL_SUCCESS;
    MMAL_BUFFER_HEADER_T *new_buffer;

    new_buffer = mmal_queue_get(pool_h264encoder->queue);

    if (new_buffer) status = mmal_port_send_buffer(port, new_buffer);
    if (!new_buffer || status != MMAL_SUCCESS) error("Could not send buffers to port");
  }

}

int main (int argc, char* argv[]) {

  MMAL_STATUS_T status;
  MMAL_COMPONENT_T *camera = 0, *jpegencoder = 0, *jpegencoder2 = 0, *h264encoder = 0, *resizer = 0;
  MMAL_ES_FORMAT_T *format;
  MMAL_CONNECTION_T *con_cam_res, *con_res_jpeg, *con_cam_h264, *con_cam_jpeg;
  int max, i, fd, length;
  char readbuf[10];
  char *filename_temp;

  bcm_host_init();
  
  //
  // read arguments
  //
  unsigned char of_set = 0;
  for(i=1; i<argc; i++) {
    if(strcmp(argv[i], "-w") == 0) {
      i++;
      width = atoi(argv[i]);
    }
    else if(strcmp(argv[i], "-h") == 0) {
      i++;
      height = atoi(argv[i]);
    }
    else if(strcmp(argv[i], "-q") == 0) {
      i++;
      quality = atoi(argv[i]);
    }
    else if(strcmp(argv[i], "-d") == 0) {
      i++;
      divider = atoi(argv[i]);
    }
    else if(strcmp(argv[i], "-of") == 0) {
      i++;
      jpeg_filename = argv[i];
      of_set = 1;
    }
    else if(strcmp(argv[i], "-if") == 0) {
      i++;
      jpeg2_filename = argv[i];
      of_set = 1;
    }
    else if(strcmp(argv[i], "-cf") == 0) {
      i++;
      pipe_filename = argv[i];
    }
    else if(strcmp(argv[i], "-vf") == 0) {
      i++;
      h264_filename = argv[i];
    }
    else if(strcmp(argv[i], "-sf") == 0) {
      i++;
      status_filename = argv[i];
    }
    else error("Invalid arguments");
  }
  if(!of_set) error("Output file not specified");

  //
  // create camera
  //
  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);
  if(status != MMAL_SUCCESS) error("Could not create camera");
  status = mmal_port_enable(camera->control, camera_control_callback);
  if(status != MMAL_SUCCESS) error("Could not enable camera control port");

  MMAL_PARAMETER_CAMERA_CONFIG_T cam_config = {
    {MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config)},
    .max_stills_w = 2592,
    .max_stills_h = 1944,
    .stills_yuv422 = 0,
    .one_shot_stills = 1,
    .max_preview_video_w = 1920,
    .max_preview_video_h = 1080,
    .num_preview_video_frames = 3,
    .stills_capture_circular_buffer_height = 0,
    .fast_preview_resume = 0,
    .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
  };
  mmal_port_parameter_set(camera->control, &cam_config.hdr);
  
  format = camera->output[0]->format;
  format->es->video.width = 1920;
  format->es->video.height = 1080;
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = 1920;
  format->es->video.crop.height = 1080;
  format->es->video.frame_rate.num = 30;
  format->es->video.frame_rate.den = 1;
  status = mmal_port_format_commit(camera->output[0]);
  if(status != MMAL_SUCCESS) error("Coult not set preview format");

  format = camera->output[1]->format;
  format->encoding_variant = MMAL_ENCODING_I420;
  format->encoding = MMAL_ENCODING_OPAQUE;
  format->es->video.width = 1920;
  format->es->video.height = 1080;
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = 1920;
  format->es->video.crop.height = 1080;
  format->es->video.frame_rate.num = 30;
  format->es->video.frame_rate.den = 1;
  status = mmal_port_format_commit(camera->output[1]);
  if(status != MMAL_SUCCESS) error("Could not set video format");
  if(camera->output[1]->buffer_num < 3)
    camera->output[1]->buffer_num = 3;
  
  format = camera->output[2]->format;
  format->encoding = MMAL_ENCODING_OPAQUE;
  format->es->video.width = 2592;
  format->es->video.height = 1944;
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = 2592;
  format->es->video.crop.height = 1944;
  format->es->video.frame_rate.num = 1;
  format->es->video.frame_rate.den = 1;
  status = mmal_port_format_commit(camera->output[2]);
  if(status != MMAL_SUCCESS) error("Could not set still format");
  if(camera->output[2]->buffer_num < 3)
    camera->output[2]->buffer_num = 3;

  status = mmal_component_enable(camera);
  if(status != MMAL_SUCCESS) error("Could not enable camera");
  
  //
  // create jpeg-encoder
  //
  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &jpegencoder);
  if(status != MMAL_SUCCESS && status != MMAL_ENOSYS) error("Could not create image encoder");

  mmal_format_copy(jpegencoder->output[0]->format, jpegencoder->input[0]->format);
  jpegencoder->output[0]->format->encoding = MMAL_ENCODING_JPEG;
  jpegencoder->output[0]->buffer_size = jpegencoder->output[0]->buffer_size_recommended;
  if(jpegencoder->output[0]->buffer_size < jpegencoder->output[0]->buffer_size_min)
    jpegencoder->output[0]->buffer_size = jpegencoder->output[0]->buffer_size_min;
  jpegencoder->output[0]->buffer_num = jpegencoder->output[0]->buffer_num_recommended;
  if(jpegencoder->output[0]->buffer_num < jpegencoder->output[0]->buffer_num_min)
    jpegencoder->output[0]->buffer_num = jpegencoder->output[0]->buffer_num_min;
  status = mmal_port_format_commit(jpegencoder->output[0]);
  if(status != MMAL_SUCCESS) error("Could not set image format");
  status = mmal_port_parameter_set_uint32(jpegencoder->output[0], MMAL_PARAMETER_JPEG_Q_FACTOR, quality);
  if(status != MMAL_SUCCESS) error("Could not set jpeg quality");

  status = mmal_component_enable(jpegencoder);
  if(status != MMAL_SUCCESS) error("Could not enable image encoder");
  pool_jpegencoder = mmal_port_pool_create(jpegencoder->output[0], jpegencoder->output[0]->buffer_num, jpegencoder->output[0]->buffer_size);
  if(!pool_jpegencoder) error("Could not create image buffer pool");

  //
  // create second jpeg-encoder
  //
  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &jpegencoder2);
  if(status != MMAL_SUCCESS && status != MMAL_ENOSYS) error("Could not create image encoder 2");

  mmal_format_copy(jpegencoder2->output[0]->format, jpegencoder2->input[0]->format);
  jpegencoder2->output[0]->format->encoding = MMAL_ENCODING_JPEG;
  jpegencoder2->output[0]->buffer_size = jpegencoder2->output[0]->buffer_size_recommended;
  if(jpegencoder2->output[0]->buffer_size < jpegencoder2->output[0]->buffer_size_min)
    jpegencoder2->output[0]->buffer_size = jpegencoder2->output[0]->buffer_size_min;
  jpegencoder2->output[0]->buffer_num = jpegencoder2->output[0]->buffer_num_recommended;
  if(jpegencoder2->output[0]->buffer_num < jpegencoder2->output[0]->buffer_num_min)
    jpegencoder2->output[0]->buffer_num = jpegencoder2->output[0]->buffer_num_min;
  status = mmal_port_format_commit(jpegencoder2->output[0]);
  if(status != MMAL_SUCCESS) error("Could not set image format 2");
  status = mmal_port_parameter_set_uint32(jpegencoder2->output[0], MMAL_PARAMETER_JPEG_Q_FACTOR, 85);
  if(status != MMAL_SUCCESS) error("Could not set jpeg quality 2");

  status = mmal_component_enable(jpegencoder2);
  if(status != MMAL_SUCCESS) error("Could not enable image encoder 2");
  pool_jpegencoder2 = mmal_port_pool_create(jpegencoder2->output[0], jpegencoder2->output[0]->buffer_num, jpegencoder2->output[0]->buffer_size);
  if(!pool_jpegencoder2) error("Could not create image buffer pool 2");

  //
  // create h264-encoder
  //
  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &h264encoder);
  if(status != MMAL_SUCCESS && status != MMAL_ENOSYS) error("Could not create video encoder");

  mmal_format_copy(h264encoder->output[0]->format, h264encoder->input[0]->format);
  h264encoder->output[0]->format->encoding = MMAL_ENCODING_H264;
  h264encoder->output[0]->format->bitrate = 17000000;
  h264encoder->output[0]->buffer_size = h264encoder->output[0]->buffer_size_recommended;
  if(h264encoder->output[0]->buffer_size < h264encoder->output[0]->buffer_size_min)
    h264encoder->output[0]->buffer_size = h264encoder->output[0]->buffer_size_min;
  h264encoder->output[0]->buffer_num = h264encoder->output[0]->buffer_num_recommended;
  if(h264encoder->output[0]->buffer_num < h264encoder->output[0]->buffer_num_min)
    h264encoder->output[0]->buffer_num = h264encoder->output[0]->buffer_num_min;
  status = mmal_port_format_commit(h264encoder->output[0]);
  if(status != MMAL_SUCCESS) error("Could not set video format");

  MMAL_PARAMETER_UINT32_T param2 = {{ MMAL_PARAMETER_VIDEO_ENCODE_INITIAL_QUANT, sizeof(param2)}, 25};
  status = mmal_port_parameter_set(h264encoder->output[0], &param2.hdr);
  if(status != MMAL_SUCCESS) error("Could not set video quantisation");

  MMAL_PARAMETER_UINT32_T param3 = {{ MMAL_PARAMETER_VIDEO_ENCODE_QP_P, sizeof(param3)}, 31};
  status = mmal_port_parameter_set(h264encoder->output[0], &param3.hdr);
  if(status != MMAL_SUCCESS) error("Could not set video quantisation");

  MMAL_PARAMETER_VIDEO_PROFILE_T param4;
  param4.hdr.id = MMAL_PARAMETER_PROFILE;
  param4.hdr.size = sizeof(param4);
  param4.profile[0].profile = MMAL_VIDEO_PROFILE_H264_HIGH;
  param4.profile[0].level = MMAL_VIDEO_LEVEL_H264_4;
  status = mmal_port_parameter_set(h264encoder->output[0], &param4.hdr);
  if(status != MMAL_SUCCESS) error("Could not set video port format");

  status = mmal_port_parameter_set_boolean(h264encoder->input[0], MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT, 1);
  if(status != MMAL_SUCCESS) error("Could not set immutable flag");

  status = mmal_port_parameter_set_boolean(h264encoder->output[0], MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, 0);
  if(status != MMAL_SUCCESS) error("Could not set inline flag");

  //
  // create image-resizer
  //
  status = mmal_component_create("vc.ril.resize", &resizer);
  if(status != MMAL_SUCCESS && status != MMAL_ENOSYS) error("Could not create image resizer");
  
  format = resizer->output[0]->format;
  format->es->video.width = width;
  format->es->video.height = height;
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = width;
  format->es->video.crop.height = height;
  format->es->video.frame_rate.num = 30;
  format->es->video.frame_rate.den = 1;
  status = mmal_port_format_commit(resizer->output[0]);
  if(status != MMAL_SUCCESS) error("Could not set image resizer output");
  
  status = mmal_component_enable(resizer);
  if(status != MMAL_SUCCESS) error("Could not enable image resizer");

  //
  // connect
  //
  status = mmal_connection_create(&con_cam_res, camera->output[0], resizer->input[0], MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
  if(status != MMAL_SUCCESS) error("Could not create connection camera -> resizer");
  status = mmal_connection_enable(con_cam_res);
  if(status != MMAL_SUCCESS) error("Could not enable connection camera -> resizer");

  status = mmal_connection_create(&con_res_jpeg, resizer->output[0], jpegencoder->input[0], MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
  if(status != MMAL_SUCCESS) error("Could not create connection resizer -> encoder");
  status = mmal_connection_enable(con_res_jpeg);
  if(status != MMAL_SUCCESS) error("Could not enable connection resizer -> encoder");

  status = mmal_port_enable(jpegencoder->output[0], jpegencoder_buffer_callback);
  if(status != MMAL_SUCCESS) error("Could not enable jpeg port");
  max = mmal_queue_length(pool_jpegencoder->queue);
  for(i=0;i<max;i++) {
    MMAL_BUFFER_HEADER_T *jpegbuffer = mmal_queue_get(pool_jpegencoder->queue);

    if(!jpegbuffer) error("Could not create jpeg buffer header");
    status = mmal_port_send_buffer(jpegencoder->output[0], jpegbuffer);
    if(status != MMAL_SUCCESS) error("Could not send buffers to jpeg port");
  }

  status = mmal_connection_create(&con_cam_jpeg, camera->output[2], jpegencoder2->input[0], MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
  if(status != MMAL_SUCCESS) error("Could not create connection camera -> encoder");
  status = mmal_connection_enable(con_cam_jpeg);
  if(status != MMAL_SUCCESS) error("Could not enable connection camera -> encoder");

  status = mmal_port_enable(jpegencoder2->output[0], jpegencoder2_buffer_callback);
  if(status != MMAL_SUCCESS) error("Could not enable jpeg port 2");
  max = mmal_queue_length(pool_jpegencoder2->queue);
  for(i=0;i<max;i++) {
    MMAL_BUFFER_HEADER_T *jpegbuffer2 = mmal_queue_get(pool_jpegencoder2->queue);

    if(!jpegbuffer2) error("Could not create jpeg buffer header 2");
    status = mmal_port_send_buffer(jpegencoder2->output[0], jpegbuffer2);
    if(status != MMAL_SUCCESS) error("Could not send buffers to jpeg port 2");
  }

  //
  // run
  //
  if(pipe_filename != 0) printf("MJPEG streaming, ready to receive commands\n");
  else printf("MJPEG streaming\n");

  struct sigaction action;
  memset(&action, 0, sizeof(struct sigaction));
  action.sa_handler = term;
  sigaction(SIGTERM, &action, NULL);
  sigaction(SIGINT, &action, NULL);
  
  if(status_filename != 0) {
    status_file = fopen(status_filename, "w");
    if(!status_file) error("Could not open/create status-file");
    fprintf(status_file, "ready");
    fclose(status_file);
  }
  
  while(running) {
    if(pipe_filename != 0) {
    
      fd = open(pipe_filename, O_RDONLY | O_NONBLOCK);
      if(fd < 0) error("Could not open PIPE");
      fcntl(fd, F_SETFL, 0);
      length = read(fd, readbuf, 10);
      close(fd);

      if(length) {
        if((readbuf[0]=='c') && (readbuf[1]=='a')) {
          if(readbuf[3]=='1') {
            status = mmal_component_enable(h264encoder);
            if(status != MMAL_SUCCESS) error("Could not enable h264encoder");
            pool_h264encoder = mmal_port_pool_create(h264encoder->output[0], h264encoder->output[0]->buffer_num, h264encoder->output[0]->buffer_size);
            if(!pool_h264encoder) error("Could not create pool");
            status = mmal_connection_create(&con_cam_h264, camera->output[1], h264encoder->input[0], MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
            if(status != MMAL_SUCCESS) error("Could not create connecton camera -> video converter");
            status = mmal_connection_enable(con_cam_h264);
            if(status != MMAL_SUCCESS) error("Could not enable connection camera -> video converter");
            h264output_file = fopen(h264_filename, "wb");
            if(!h264output_file) error("Could not open/create video-file");
            status = mmal_port_enable(h264encoder->output[0], h264encoder_buffer_callback);
            if(status != MMAL_SUCCESS) error("Could not enable video port");
            max = mmal_queue_length(pool_h264encoder->queue);
            for(i=0;i<max;i++) {
              MMAL_BUFFER_HEADER_T *h264buffer = mmal_queue_get(pool_h264encoder->queue);
              if(!h264buffer) error("Could not create video pool header");
              status = mmal_port_send_buffer(h264encoder->output[0], h264buffer);
              if(status != MMAL_SUCCESS) error("Could not send buffers to video port");
            }
            mmal_port_parameter_set_boolean(camera->output[1], MMAL_PARAMETER_CAPTURE, 1);
            if(status != MMAL_SUCCESS) error("Could not start capture");
            printf("Capturing started\n");
            if(status_filename != 0) {
              status_file = fopen(status_filename, "w");
              fprintf(status_file, "video");
              fclose(status_file);
            }
          }
          else {
            mmal_port_parameter_set_boolean(camera->output[1], MMAL_PARAMETER_CAPTURE, 0);
            if(status != MMAL_SUCCESS) error("Could not stop capture");
            status = mmal_port_disable(h264encoder->output[0]);
            if(status != MMAL_SUCCESS) error("Could not disable video port");
            status = mmal_connection_destroy(con_cam_h264);
            if(status != MMAL_SUCCESS) error("Could not destroy connection camera -> video encoder");
            mmal_port_pool_destroy(h264encoder->output[0], pool_h264encoder);
            if(status != MMAL_SUCCESS) error("Could not destroy video buffer pool");
            status = mmal_component_disable(h264encoder);
            if(status != MMAL_SUCCESS) error("Could not disable video converter");
            fclose(h264output_file);
            h264output_file = NULL;
            printf("Capturing stopped\n");
            if(status_filename != 0) {
              status_file = fopen(status_filename, "w");
              fprintf(status_file, "ready");
              fclose(status_file);
            }
          }
        }
        else if((readbuf[0]=='i') && (readbuf[1]=='m')) {
          asprintf(&filename_temp, jpeg2_filename, image2_cnt);
          jpegoutput2_file = fopen(filename_temp, "wb");
          if(!jpegoutput2_file) error("Could not open/create image-file");
          status = mmal_port_parameter_set_boolean(camera->output[2], MMAL_PARAMETER_CAPTURE, 1);
          if(status != MMAL_SUCCESS) error("Could not start image capture");
          printf("Capturing image\n");
          if(status_filename != 0) {
            status_file = fopen(status_filename, "w");
            fprintf(status_file, "image");
            fclose(status_file);
          }
        }
      }

    }
    usleep(100);
  }
  
  printf("SIGINT/SIGTERM received, stopping\n");
  
  //
  // tidy up
  //
  mmal_port_disable(jpegencoder->output[0]);
  mmal_connection_destroy(con_cam_res);
  mmal_connection_destroy(con_res_jpeg);
  mmal_port_pool_destroy(jpegencoder->output[0], pool_jpegencoder);
  mmal_component_disable(jpegencoder);
  mmal_component_disable(camera);
  mmal_component_destroy(jpegencoder);
  mmal_component_destroy(h264encoder);
  mmal_component_destroy(camera);

  return 0;

}
