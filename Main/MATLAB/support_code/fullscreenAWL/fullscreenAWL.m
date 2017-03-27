function [frame_java] = fullscreenAWL(fipath)
%FULLSCREENAWL Display fullscreen true colour images
%   Based on FULLSCREEN(C,N) displays bit map image C on all monitors. 
%   Image matrix C must be the exact resolution of the output screen since 
%   no scaling implemented. If fullscreen is activated on the same display
%   as the MATLAB window, use ALT-TAB to switch back.
%
%   If FULLSCREENAWL(C) is called the second time, the screen will update
%   with the new image.
%
%   Use CLOSESCREENAWL() to exit fullscreen.
%
%   Requires Matlab 7.x (uses Java Virtual Machine), and has been tested on
%   Linux and Windows platforms.
%
%   Originally written by Pithawat Vachiramon
%
%   Update (07/29/14) by AWL
%   Update (23/3/09):
%   - Uses temporary bitmap file to speed up drawing process.
%   - Implemeted a fix by Alejandro Camara Iglesias to solve issue with
%   non-exclusive-fullscreen-capable screens.

% Set up variabls
% JFrame object
global frame_java;
% ImageIcon object
global icon_java;
% graphics environment
ge = java.awt.GraphicsEnvironment.getLocalGraphicsEnvironment();
% screen device object
gds = ge.getScreenDevices();
% image
buff_image = javax.imageio.ImageIO.read(java.io.File(fipath));
    
if ~isequal(class(frame_java), 'javax.swing.JFrame') || ...
        ~frame_java.isShowing()
    frame_java = javax.swing.JFrame(gds(1).getDefaultConfiguration());
    frame_java.setUndecorated(true);
    icon_java = javax.swing.ImageIcon(buff_image);
    label = javax.swing.JLabel(icon_java);
    frame_java.getContentPane.add(label);
    gds(1).setFullScreenWindow(frame_java);
    frame_java.setLocation(1600, 0);
    %frame_java.setVisible(true)
    %frame_java.setAlwaysOnTop(true)
else
    icon_java.setImage(buff_image);
end
frame_java.pack
frame_java.repaint
frame_java.show
