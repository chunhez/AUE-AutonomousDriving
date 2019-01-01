clc
clear all
close all

a=arduino('COM3','Uno','Libraries','Servo');
Angle=servo(a,'D10','MinPulseDuration',1e-3,'MaxPulseDuration',2e-3);
v=servo(a,'D11','MinPulseDuration',1e-3,'MaxPulseDuration',2e-3);
%%
webcamlist
cam=webcam('USB_camera');
cam.Resolution='320x240';
i=1;
pause(1)
         PKK1(1)=0;
         PKK1(2)=0;
         PKK2(1)=0;
         PKK2(2)=0;
         COUNT=1;
         Qslope=1;
         Qphi=0.1;
         Rslope=0.0001;
         Rphi=0.45;        
 vel=0.569446;
 writePosition(v,vel);

while i
    t=tic;    
img=snapshot(cam);   
p=img;%original image
shape = size(p);

rlim=210;
glim=210;
blim=210;
[x,y]=find(p(:,:,1)<rlim|p(:,:,2)<glim|p(:,:,3)<blim);
toc(t)
for o=1:length(x)
    p(x(o),y(o),:)=[0,0,0];
end

p1=rgb2gray(p);
BW1=edge(p1,'canny',[0.25 0.6]);%detect edges
target_x=0.99; 
a=[shape(1)*0.5, shape(1)*0.5,shape(1)*target_x,shape(1)*target_x];
b=[0,shape(2),shape(2),0];
bw=roipoly(p,b,a);
BW=(BW1(:,:,1)&bw);
% HOUGH TRANSFORM
[H,theta,rho]=hough(BW);
P = houghpeaks(H,3);
lines = houghlines(BW,theta,rho,P,'FillGap',20,'MinLength',5);

  for k = 2:length(lines)
       if abs(lines(k).rho-lines(1).rho)<70&&abs(lines(k).theta-lines(1).theta)<10      
            m1(1,:)=lines(1).point1;
            m1(k,:)=lines(k).point1;
            n1(1,:)=lines(1).point2; 
            n1(k,:)=lines(k).point2;
      else
            m1(1,:)=lines(1).point1;
            m2(k,:)=lines(k).point1;
            n1(1,:)=lines(1).point2; 
            n2(k,:)=lines(k).point2;
      end
  end
    
  if exist('m1')&&exist('m2')&&exist('n1')&&exist('n2')    
         mn1=[m1;n1];
         mn2=[m2;n2];
         mn1(all(mn1==0,2),:)=[];
         mn2(all(mn2==0,2),:)=[];
                         
         kk1=polyfit(mn1(:,1),mn1(:,2),1);
         kk2=polyfit(mn2(:,1),mn2(:,2),1);
                  
        if kk1(1)>0.8||kk1(1)<-0.8
         ymn1=mn1(:,1)*kk1(1,1)+kk1(1,2);
         ymn2=mn2(:,1)*kk2(1,1)+kk2(1,2);      
         lclo=[(0.81*shape(1)-kk1(2))/kk1(1),0.81*shape(1)];
         rclo=[(0.81*shape(1)-kk2(2))/kk2(1),0.81*shape(1)];
        end
  end 
  
ul_1=lclo(1)
vl_1=lclo(2)
ur_1=rclo(1) 
vr_1=rclo(2)

Xl_1=(ul_1-160)
Xr_1=(ur_1-160)

xdesired=-(Xl_1+Xr_1);
zdesired=240-vl_1;
alpha=atan(xdesired/(zdesired+206));

delta=atan(2*167*sin(alpha)/(xdesired/sin(alpha)));
if delta>0.03&&delta<50
    phi=max(0.2,min(0.33,(-0.635)*delta+0.33));    
elseif delta<-0.03&&delta>-50
    phi=min(0.8,max(0.49,(-0.945)*delta+0.37));
else
    phi=0.39;
end

if i==1
    PHI=phi;
    PP=0;
end
%kalman for phi
         PPHI=PHI;
         PPRED=PP+Qphi;
         KGAIN=PPRED/(PPRED-Rphi);
         PHI=PPHI(1)+(KGAIN*(phi-PPHI(1)));
         PP=(1-KGAIN)*PPRED;
writePosition(Angle,phi);
toc(t)
clearvars -except cam i a Angle v lclo lfar rclo rfar kk1 kk2 delta vel ul_1 ur_1 PPHI PHI PPRED PP KGAIN Qphi Rphi
i=i+1;
end
clear('cam')


