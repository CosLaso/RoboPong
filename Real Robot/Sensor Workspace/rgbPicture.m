rosshutdown
rosinit
sub=rossubscriber("/camera/color/image_raw");
pause(1);
[msg2] = receive(sub,10)
image = msg2.readImage
imshow(image)