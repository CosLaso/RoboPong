function varargout = DobotGUI(varargin)
% DOBOTGUI MATLAB code for DobotGUI.fig
%      DOBOTGUI, by itself, creates a new DOBOTGUI or raises the existing
%      singleton*.
%
%      H = DOBOTGUI returns the handle to a new DOBOTGUI or the handle to
%      the existing singleton*.
%
%      DOBOTGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DOBOTGUI.M with the given input arguments.
%
%      DOBOTGUI('Property','Value',...) creates a new DOBOTGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before DobotGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to DobotGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help DobotGUI

% Last Modified by GUIDE v2.5 13-May-2022 17:07:12

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @DobotGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @DobotGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before DobotGUI is made visible.
function DobotGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to DobotGUI (see VARARGIN)

% Choose default command line output for DobotGUI
handles.output = hObject;

img=imread('estop.jpg');
axes(handles.axes2);
imshow(img);

%% CREATION OF THE ROBOT CLASS

L(1) = Link('d',0.1,'a',0,'alpha',-pi/2,'qlim',[deg2rad(-135) deg2rad(135)]);
L(2) = Link('d',0,'a',0.135,'alpha',0,'qlim',[deg2rad(-5) deg2rad(85)]);
L(3) = Link('d',0,'a',0.147,'alpha',0,'qlim',[deg2rad(-10) deg2rad(95)]);
L(4) = Link('d',0,'a',0.04,'alpha',pi/2,'qlim',[deg2rad(-90) deg2rad(90)]);


Robot = SerialLink(L);

Robot.name = 'Dobot Magician';

% Offsets so the robot plots in a good position
handles.theta_1.String = '0';
handles.theta_2.String = '-5';
handles.theta_3.String = '55.1';
handles.theta_4.String = '-50.1';

J = Robot.fkine([0 -0.0873 0.9616 -0.8857]);

handles.CurrentT1 = 0;
handles.CurrentT2 = -5;

handles.CurrentT3 = 55.1;
handles.CurrentT4 = -50.1;

% Display the end effector position on the GUI
handles.pos_x.String = num2str(floor(J(1,4)));
handles.pos_y.String = num2str(floor(J(2,4)));
handles.pos_z.String = num2str(floor(J(3,4)));

axes(handles.axes1)
Robot.plot([0 -0.0873 0.9616 -0.8857]);
view(45,30);

handles.Robot = Robot;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes DobotGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = DobotGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function theta_1_Callback(hObject, eventdata, handles)
% hObject    handle to theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta_1 as text
%        str2double(get(hObject,'String')) returns contents of theta_1 as a double


% --- Executes during object creation, after setting all properties.
function theta_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function theta_2_Callback(hObject, eventdata, handles)
% hObject    handle to theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta_2 as text
%        str2double(get(hObject,'String')) returns contents of theta_2 as a double


% --- Executes during object creation, after setting all properties.
function theta_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function theta_3_Callback(hObject, eventdata, handles)
% hObject    handle to theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta_3 as text
%        str2double(get(hObject,'String')) returns contents of theta_3 as a double


% --- Executes during object creation, after setting all properties.
function theta_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function theta_4_Callback(hObject, eventdata, handles)
% hObject    handle to theta_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta_4 as text
%        str2double(get(hObject,'String')) returns contents of theta_4 as a double


% --- Executes during object creation, after setting all properties.
function theta_4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on button press in forward.
function forward_Callback(hObject, eventdata, handles)
% hObject    handle to forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

Th_1 = str2double(handles.theta_1.String)*pi/180;
Th_2 = str2double(handles.theta_2.String)*pi/180;
Th_3 = str2double(handles.theta_3.String)*pi/180;
Th_4 = str2double(handles.theta_4.String)*pi/180;



handles.Robot.plot([Th_1 Th_2 Th_3 Th_4]);

T = handles.Robot.fkine([Th_1 Th_2 Th_3 Th_4]);
handles.CurrentT1 = Th_1;
handles.CurrentT2 = Th_2;
handles.CurrentT3 = Th_3;
handles.CurrentT4 = Th_4;
num2str(T(1,4))

handles.pos_x.String
handles.pos_x.String = num2str(T(1,4));
handles.pos_y.String = num2str(T(2,4));
handles.pos_z.String = num2str(T(3,4));



function pos_x_Callback(hObject, eventdata, handles)
% hObject    handle to pos_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pos_x as text
%        str2double(get(hObject,'String')) returns contents of pos_x as a double


% --- Executes during object creation, after setting all properties.
function pos_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pos_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pos_y_Callback(hObject, eventdata, handles)
% hObject    handle to pos_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pos_y as text
%        str2double(get(hObject,'String')) returns contents of pos_y as a double


% --- Executes during object creation, after setting all properties.
function pos_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pos_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_Inverse.
function btn_Inverse_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
X = str2double(handles.pos_x.String);
Y = str2double(handles.pos_y.String);
Z = str2double(handles.pos_z.String);

T = [ 1 0 0 X;
      0 1 0 Y;
      0 0 1 Z;
      0 0 0 1];
  
  
J = handles.Robot.ikine(T, [0 handles.CurrentT2 handles.CurrentT3 handles.CurrentT4], [1 1 1 0 0 0]) * 180/pi;
handles.theta_1.String = num2str(floor(J(1)));
handles.theta_2.String = num2str(floor(J(2)));
handles.theta_3.String = num2str(floor(J(3)));
handles.theta_4.String = num2str(floor(J(4)));


handles.Robot.plot(J*pi/180);



function pos_z_Callback(hObject, eventdata, handles)
% hObject    handle to pos_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pos_z as text
%        str2double(get(hObject,'String')) returns contents of pos_z as a double


% --- Executes during object creation, after setting all properties.
function pos_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pos_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double


% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit10_Callback(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit10 as text
%        str2double(get(hObject,'String')) returns contents of edit10 as a double


% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in inverse.
function inverse_Callback(hObject, eventdata, handles)
% hObject    handle to inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on slider movement.
function theta_1_slider_Callback(hObject, eventdata, handles)
% hObject    handle to theta_1_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Joint 1

Th_1 = str2double(handles.theta_1.String)*pi/180;
Th_2 = str2double(handles.theta_2.String)*pi/180;
Th_3 = str2double(handles.theta_3.String)*pi/180;
Th_4 = str2double(handles.theta_4.String)*pi/180;

a=get(handles.theta_1_slider, 'Value');
x=0:10:360;

% Manipulation of q1 based on the theta_1_slider location
handles.Robot.plot([(Th_1+7*a) Th_2 Th_3 Th_4]);

T = handles.Robot.fkine([Th_1+7*a Th_2 Th_3 Th_4]);

handles.CurrentT1 = Th_1;
handles.CurrentT2 = Th_2;
handles.CurrentT3 = Th_3;
handles.CurrentT4 = Th_4;

handles.theta_1.String
c=(str2double(handles.theta_1.String)*180/pi)

Angle2=(Th_1+7*a)*180/pi
handles.theta_1.String=Angle2;

handles.pos_x.String = num2str(T(1,4));
handles.pos_y.String = num2str(T(2,4));
handles.pos_z.String = num2str(T(3,4));



% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function theta_1_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_1_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function theta_2_slider_Callback(hObject, eventdata, handles)
% hObject    handle to theta_2_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
Th_1 = str2double(handles.theta_1.String)*pi/180;
Th_2 = str2double(handles.theta_2.String)*pi/180;
Th_3 = str2double(handles.theta_3.String)*pi/180;
Th_4 = str2double(handles.theta_4.String)*pi/180;

b=get(handles.theta_2_slider, 'Value');
x=0:10:360;

% Manipulation of q2 based on the slider input
handles.Robot.plot([Th_1 Th_2+7*b Th_3 Th_4]); % TH_1, TH_3, TH_4 obtained via the method shown above

T = handles.Robot.fkine([Th_1 Th_2+7*b Th_3 Th_4]);

handles.CurrentT1 = Th_1;
handles.CurrentT2 = Th_2;
handles.CurrentT3 = Th_3;
handles.CurrentT4 = Th_4;

handles.theta_2.String
c2=(str2double(handles.theta_2.String)*180/pi)

% Displaying the right angle for q2 in the write GUI popup window
Angle2b=(Th_2+7*b)*180/pi
handles.theta_2.String=Angle2b;

handles.pos_x.String = num2str(T(1,4));
handles.pos_y.String = num2str(T(2,4));
handles.pos_z.String = num2str(T(3,4));

% --- Executes during object creation, after setting all properties.
function theta_2_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_2_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function theta_3_slider_Callback(hObject, eventdata, handles)
% hObject    handle to theta_3_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
Th_1 = str2double(handles.theta_1.String)*pi/180;
Th_2 = str2double(handles.theta_2.String)*pi/180;
Th_3 = str2double(handles.theta_3.String)*pi/180;
Th_4 = str2double(handles.theta_4.String)*pi/180;

c3=get(handles.theta_3_slider, 'Value');
x=0:10:360;

% Manipulation of q3 based on the slider input
handles.Robot.plot([Th_1 Th_2 Th_3+7*c3 Th_4]);

T = handles.Robot.fkine([Th_1 Th_2 Th_3+7*c3  Th_4]);

handles.CurrentT1 = Th_1;
handles.CurrentT2 = Th_2;
handles.CurrentT3 = Th_3;
handles.CurrentT4 = Th_4;

handles.theta_3.String
c4=(str2double(handles.theta_3.String)*180/pi)

% Displaying the right angle for q3 in the write GUI popup window
Angle3=(Th_2+7*c3)*180/pi
handles.theta_3.String=Angle3;

handles.pos_x.String = num2str(T(1,4));
handles.pos_y.String = num2str(T(2,4));
handles.pos_z.String = num2str(T(3,4));


% --- Executes during object creation, after setting all properties.
function theta_3_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_3_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function theta_4_slider_Callback(hObject, eventdata, handles)
% hObject    handle to theta_4_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
Th_1 = str2double(handles.theta_1.String)*pi/180;
Th_2 = str2double(handles.theta_2.String)*pi/180;
Th_3 = str2double(handles.theta_3.String)*pi/180;
Th_4 = str2double(handles.theta_4.String)*pi/180;

h=get(handles.theta_4_slider, 'Value');
x=0:10:360;

% Manipulation of q4 based on the slider input
handles.Robot.plot([Th_1 Th_2 Th_3 Th_4+7*h]);

T = handles.Robot.fkine([Th_1 Th_2 Th_3 Th_4+7*h]);

handles.CurrentT1 = Th_1;
handles.CurrentT2 = Th_2;
handles.CurrentT3 = Th_3;
handles.CurrentT4 = Th_4;

handles.theta_4.String
k=(str2double(handles.theta_4.String)*180/pi)

Angle4=(Th_4+7*h)*180/pi
handles.theta_4.String=Angle4;

handles.pos_x.String = num2str(T(1,4));
handles.pos_y.String = num2str(T(2,4));
handles.pos_z.String = num2str(T(3,4));

% --- Executes during object creation, after setting all properties.
function theta_4_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_4_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in demo.
function demo_Callback(hObject, eventdata, handles)
% hObject    handle to demo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
L1 = Link('d',0.1,'a',0,'alpha',-pi/2,'qlim',[deg2rad(-135) deg2rad(135)]);
L2 = Link('d',0,'a',0.135,'alpha',0,'qlim',[deg2rad(-5) deg2rad(85)]);
L3 = Link('d',0,'a',0.147,'alpha',0,'qlim',[deg2rad(-10) deg2rad(95)]);
L4 = Link('d',0,'a',0.04,'alpha',pi/2,'qlim',[deg2rad(-90) deg2rad(90)]);
robot = SerialLink([L1 L2 L3 L4],'name','Denso VM6083G');

stepRads = deg2rad(20);
 qlim = robot.qlim;
% qlim=[[deg2rad(-135) deg2rad(135)] [deg2rad(-135) deg2rad(135)]  [deg2rad(-135) deg2rad(135)] [deg2rad(-135) deg2rad(135)]]
pointCloudeSize = prod(floor((qlim(1:4,2)-qlim(1:4,1))/stepRads + 1));
pointCloud = zeros(pointCloudeSize,3);
counter = 1;
tic

for q1 = qlim(1,1):stepRads:qlim(1,2)
    for q2 = qlim(2,1):stepRads:qlim(2,2)
        for q3 = qlim(3,1):stepRads:qlim(3,2)
            for q4 = qlim(4,1):stepRads:qlim(4,2)
                        q = [q1,q2,q3,q4];
                        tr = robot.fkine(q);
                        view(45,30);
                        pointCloud(counter,:) = tr(1:3,4)';
                        counter = counter + 1; 
                        if mod(counter/pointCloudeSize * 100,1) == 0
                            display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                        end 
            end
        end
    end
end
view(45,30);

plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
