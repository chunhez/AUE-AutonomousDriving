clc;
close all;
clear all;

delete(instrfindall)
ipA = '198.21.196.20';
portA=9090;

ipB='198.21.239.44';
portB=9091;

udpA=udp(ipB,portB,'LocalPort',portA);
fopen(udpA);

check1=0;
check2=0;

cam = webcam(2); 
cam.Resolution = '320x240'; 
videoFrame = snapshot(cam); 
frameSize = size(videoFrame); 
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]); 
detector1 = vision.CascadeObjectDetector('newStopSign.xml');
detector2 = vision.CascadeObjectDetector('newSchoolSign.xml');
elapsedTime_school=0;
elapsedTime_stop=0;
itr=0;
while 1
    time=tic;
    videoFrame = snapshot(cam); 
    videoFrameGray = rgb2gray(videoFrame); 
    bbox1 = step(detector1,videoFrameGray);
    bbox2 = step(detector2,videoFrameGray);
    if (isempty(bbox1)~=1) 
            videoFrame = insertObjectAnnotation(videoFrame,'rectangle',bbox1,'stop sign'); 
            step(videoPlayer, videoFrame); 
            check1=check1+1;
            elapsedTime_stop=elapsedTime_stop+itr;
            if ((check1>=45) && (elapsedTime_stop>2))
                display(check1);
                display(elapsedTime_stop);
                fprintf(udpA,'%d\n',1);
                elapsedTime_stop=0;
                elapsedTime_school=0;
                check1=0;
                check2=0;
            end
    elseif (isempty(bbox2)~=1) 
            videoFrame = insertObjectAnnotation(videoFrame,'rectangle',bbox2,'school sign'); 
            step(videoPlayer, videoFrame); 
            check2=check2+1;
            elapsedTime_school=elapsedTime_school+itr;
            if ((check2>=10) && (elapsedTime_school>0.6))
                display(check2);
                display(elapsedTime_school);
                fprintf(udpA,'%d\n',2);
                elapsedTime_school=0;
                elapsedTime_stop=0;
                check2=0;
                check1=0;
            end
    else
         step(videoPlayer, videoFrame); 
    end
    itr=toc(time);
    
    clear bbox1 bbox2;
end