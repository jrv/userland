RaspiMJPEG is an OpenMAX-Application based on the mmal-library, which is comparable to RaspiVid. Both applications save the recording formated as H264 into a file. But instead of showing the preview on a screen, RaspiMJPEG streams the preview as MJPEG into a file. The update-rate and the size of the preview are customizable with parameters and independent of the recording, which is always 1080p 30fps. Once started, the application receives commands with a unix-pipe and showes its status on stdout and writes it into a status-file. The program terminates itself after receiving a SIGINT or SIGTERM

Usage as MJPEG-Streamer:
raspimjpeg -w 320 -h 240 -d 5 -of /path/to/image.jpg

This command creates the image "path/to/image.jpg" with the size 320x240 px and updates it every time, 5 frames are captured, which gives (30/5=) 6fps. This way used, RaspiMJPEG serves as MJPEG-streamer and the preview can be showed on a website for example. To achieve a high update-rate (e.g. "-d 1"), it is recommended to save the file into the ram and not on the sd-card (e.g. "-of /dev/shm/image.jpg").


Usages as MJPEG-Streamer with capture:
raspimjpeg -w 320 -h 240 -d 5 -of /path/to/image.jpg -cf /path/to/pipe -vf /path/to/video.h264

This command does the same as the one above, but it is also listening on the pipe, defined with the parametet -c. If another program writes "ca 1" into the pipe (shell: echo "ca 1" > /path/to/pipe), the application continues with the MJPEG-stream, but starts also a H264-capture 1080p 30fps into the defined file. The capture is stopped with the command "ca 0" via pipe. A new capture overwrites the defined file. To make MJPEG and H264 possible, it is recommended to write the image file only into the ram, as described above.


Usage as MJPEG-streamer with capture and status-output:
raspimjpeg -w 320 -h 240 -d 5 -of /path/to/image.jpg -cf /path/to/pipe -vf /path/to/video.h264 -sf /path/to/textfile.txt

Until now, RaspiMJPEG wrote its status into stdout/stderr. With this new command, the status is also written into a textfile (no logging, just the newest status). Possible messages and their meanings are:
ready --> MJPEG is streaming, not capturing
capture --> MJPEG is streaming and H264 is capturing
errror --> An error occured and the application terminated
