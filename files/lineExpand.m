function polygons = lineExpand(lines,r)


x1 = lines(1);
y1 = lines(2);
x2 = lines(3);
y2 = lines(4);


th = atan2(y2-y1,x2-x1);

x2_1 = x2 + r * cos(th + pi/4);
y2_1 = y2 + r * sin(th + pi/4);
x2_2 = x2 + r * cos(th - pi/4);
y2_2 = y2 + r * sin(th - pi/4);

x1_1 = x1 + r * cos(th + pi - pi/4);
y1_1 = y1 + r * sin(th + pi - pi/4);
x1_2 = x1 + r * cos(th + pi + pi/4);
y1_2 = y1 + r * sin(th + pi + pi/4);

polygons = [x2_1, y2_1, x1_1, y1_1;
            x1_1, y1_1, x1_2, y1_2;
            x1_2, y1_2, x2_2, y2_2;
            x2_2, y2_2, x2_1, y2_1];

end