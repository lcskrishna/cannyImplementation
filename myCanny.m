function myCanny()
close all;
f = imread('valve.png');
figure ; imshow(f); title('Input Image');
f = rgb2gray(f);
fd = double(f);

sigma = 2;
fd  = imgaussfilt(fd , sigma);
sX = 1.0/8.0 * [1 0 -1 ; 2 0 -2 ; 1 0 -1];
sY = sX';

gx = imfilter(fd , sX , 'conv');
gy = imfilter(fd , sY , 'conv');
figure; imshow(gx , []);title('Gradiant X');
figure; imshow(gy , []);title('Gradiant Y');


gm = sqrt(gx.^2 + gy.^2);
gd = atan2(gy , gx);

figure;
imshow(gd , []);title('Gradiant Direction');
colormap jet;

figure;
imshow(gm , []);title('Gradiant Magnitude');
[R , C] = size(f);

gLoc = gm;
for r = 2 : (R - 1)
    for c = 2 : (C - 1)
        v = gm(r ,  c);
        t = gd(r , c);
        
        t =  (t / pi ) * 180;
        newAngle = 0;
        
        if ( ( (t < 22.5) && (t > -22.5) ) || ((t > 157.5) || (t < -157.5)))
            newAngle = 0;
        end;
        if ( ( (t > 22.5) && (t < 67.5) ) || ( (t < -112.5) && (t > -157.5)))
            newAngle = 45;
        end;
        if ( ( (t > 67.5) && (t < 112.5) ) || ( (t < -67.5) && (t > -112.5) ) )
            newAngle = 90;
        end;
        if(((t > 112.5) && (t < 157.5)) || ((t < -22.5) && (t > -67.5)))
            newAngle = 135;
        end;
        
        if(newAngle == 0 &&  ( v < gm(r  , c - 1) || v  < gm(r , c + 1)))
            gLoc(r ,  c) = 0;
        end
        
        if(newAngle == 90 &&  ( v < gm(r - 1  , c ) || v  < gm(r + 1 , c )))
            gLoc(r ,  c) = 0;
        end
        
        if(newAngle == 45 &&  ( v < gm(r - 1 , c - 1) || v  < gm(r + 1, c + 1)))
            gLoc(r ,  c) = 0;
        end
        
        if(newAngle == 135 &&  ( v < gm(r + 1 , c - 1) || v  < gm(r - 1 , c + 1)))
            gLoc(r ,  c) = 0;
        end
        
    end
    
end

figure;
imshow(gLoc , []);title('Non Max Suppression Output');


gLocal = gLoc;
gLocal = uint8(gLocal);

hist= imhist(gLocal);
hist= hist/numel(gLocal(:));
figure,plot(hist);
[cdf]= cumsum(hist);
figure,plot(cdf);

%%% Alpha = 10 % of edge pixels.
%%% Beta
alpha = 0.9;beta=0.2;
thigh= 0;
tlow = 0;

for i=1:256
    if(cdf(i)> alpha)
        thigh = i;
        break;
    end
end

thigh=6;
tlow=1.2;

bEdge =  edgeLinking(gLoc , tlow ,thigh);
figure ; imshow(bEdge); title('Canny Edge');


end


%edge linking
function bEdge = edgeLinking(gLoc , tLow , tHigh)

sz = size(gLoc);
bEdge = zeros(sz(1) , sz(2));
sMap = zeros(sz(1) , sz(2));
wMap = zeros(sz(1) , sz(2));
frontier = [];
for r = 1 : sz(1)
    for c  = 1 : sz(2)
        if gLoc(r , c) >= tHigh
            frontier = [ frontier , r , c];
            bEdge(r , c) = 1;
            sMap(r , c) = 1;
        end;
        fsz = size(frontier);
        while(fsz(2) >= 2)
            x = frontier(fsz(2) - 1);%row
            y = frontier(fsz(2) );%colo
            
            %disp(x);
            %disp(y);
            frontier = frontier( 1 : fsz(2) - 2);
            if( x > 1 && gLoc(x - 1 , y) >= tLow)  %left
                bEdge(x - 1 , y) = 1;
                if(wMap(x - 1 , y) ~= 1 && sMap(x - 1 , y) ~= 1)
                    wMap(x - 1 , y) = 1;
                    frontier = [ frontier : x - 1 , y];
                end;
            end;
            if(x < sz(1) &&  gLoc(x + 1 , y) >= tLow)  % r
                bEdge(x + 1 , y) = 1;
                if(wMap(x + 1 , y) ~= 1 && sMap(x + 1 , y) ~= 1)
                    wMap(x + 1 , y) = 1;
                    frontier = [ frontier : x + 1 , y];
                end;
            end;
            
            if(y > 1 && gLoc(x  , y - 1) >= tLow)  % r
                bEdge(x  , y - 1) = 1;
                if(wMap(x , y - 1) ~= 1 && sMap(x , y - 1) ~= 1 )
                    wMap(x  , y - 1) = 1;
                    frontier = [ frontier : x  , y - 1];
                end;
            end;
            
            if(y < sz(2) && gLoc(x  , y + 1) >= tLow)  % r
                bEdge(x , y + 1) = 1;
                if(wMap(x  , y + 1) ~= 1 && sMap(x  , y + 1) ~= 1)
                    wMap(x , y + 1) = 1;
                    frontier = [ frontier : x  , y + 1];
                end;
            end;
            
            
            if(x > 1 && y > 1 && gLoc(x - 1  , y - 1) >= tLow)  % r
                bEdge(x - 1 , y - 1) = 1;
                if(wMap(x - 1 , y - 1) ~= 1 && sMap(x - 1 , y - 1) ~= 1)
                    wMap(x - 1 , y - 1) = 1;
                    frontier = [ frontier : x  - 1, y - 1];
                end;
            end;
            
            if( y < sz(2) && x > 1 && gLoc(x - 1  , y + 1) >= tLow)  % r
                bEdge(x - 1 , y + 1) = 1;
                if(wMap(x - 1 , y + 1) ~= 1 && sMap(x - 1 , y + 1) ~= 1)
                    wMap(x - 1 , y + 1) = 1;
                    frontier = [ frontier : x  - 1, y + 1];
                end;
            end
            
            if(x < sz(1) &&  y < sz(2) &&  gLoc(x + 1  , y + 1) >= tLow)  % r
                bEdge(x + 1 , y + 1) = 1;
                if(wMap(x + 1 , y + 1) ~= 1 && sMap(x + 1 , y + 1) ~= 1)
                    wMap(x + 1 , y + 1) = 1;
                    frontier = [ frontier : x  + 1, y + 1];
                end;
            end;
            
            if(x < sz(1) &&  y > 1 && gLoc(x + 1  , y - 1) >= tLow)  % r
                bEdge(x + 1 , y - 1) = 1;
                if(wMap(x + 1 , y - 1) ~= 1 && sMap(x + 1 , y - 1) ~= 1)
                    wMap(x + 1 , y - 1) = 1;
                    frontier = [ frontier : x  + 1, y - 1];
                end;
            end;
            
            fsz = size(frontier);
        end;
    end;
end;

figure; imshow(sMap ,[]); title('Strong Edge');
figure; imshow(wMap ,[]); title('Weak Edge');
end