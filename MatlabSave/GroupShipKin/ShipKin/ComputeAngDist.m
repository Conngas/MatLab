function [ ang,dist ] = ComputeAngDist( LonA,LatA,LonB,LatB )

%输入
%待求点AB的经纬度值  单位°
%输出
%待求点B到相对A点（0,0）的角度(正北为基准)和距离
%仍然存在的问题：无



e=0.0818191;
r=6378137.0;
myr=0.0;
mys=0.0;

tolat=LatB*pi/180.0;
tolon=LonB*pi/180.0;
fromlat=LatA*pi/180.0;
fromlon=LonA*pi/180.0;

%注意，下面的经纬度差一定要是弧度为单位
dltlon=(LonB-LonA)*pi/180.0;
dltlat=(LatB-LatA)*pi/180.0;

if dltlat==0
    if dltlon>0
        ang=90.0;
    else
        ang=270.0; 
    end
else
    %  tolatrat = log( tan( ( PI * 0.25 + tolat * 0.5  ) * pow( ( ( 1.0 - e *  sin( tolat ) )   / (1.0  + e * cos( tolat ) ) ),( e *0.5  ))));
    tolatrat=log(tan((pi*0.25+tolat*0.5)*power(((1.0-e*sin(tolat))/(1.0+e*cos(tolat))),(e*0.5))));
    fromlatrat=log(tan((pi*0.25+fromlat*0.5)*power(((1.0-e*sin(fromlat))/(1.0+e*cos(fromlat))),(e*0.5))));
    angletemp=atan(dltlon/(tolatrat-fromlatrat))*180/pi;
    if angletemp>0
        if dltlon>0
            ang=angletemp;
        else
            ang=180.0+angletemp;
        end
    elseif angletemp==0
        if dltlon>0
            ang=0.0;
        else
            ang=180.0;
        end
    elseif angletemp<0
        if dltlon>0
            ang=180.0-abs(angletemp);
        else
            ang=360.0-abs(angletemp);
        end
    end
    
    
end

k1=1.0+0.75*power(e,2)+45.0/16.0*power(e,4.0);
k2=0.75*power(e,2.0)+15.0/16.0*power(e,4.0);
k3=15.0/64.0*power(e,4.0);

if dltlat==0
    myr=r*cos(tolat)/power((1.0-e*e*sin(tolat)*sin(tolat)),0.5);
    mys=abs(myr*dltlon);
elseif dltlon==0
    mys=r*(1.0-e*e)*abs((k1*tolat-0.5*k2*sin(2*tolat)+0.25*k3*sin(4.0*tolat))-(k1*fromlat-0.5*k2*sin(2.0*fromlat)+0.25*k3*sin(4.0*fromlat)));
elseif ((dltlon~=0)&&(dltlat~=0))
    temp=r*(1.0-e*e)*abs((k1*tolat-0.5*k2*sin(2.0*tolat)+0.25*k3*sin(4.0*tolat))-(k1*fromlat-0.5*k2*sin(2.0*fromlat)+0.25*k3*sin(4.0*fromlat)));
    mys=abs(temp/cos(ang*pi/180));
end

dist=mys;

end

