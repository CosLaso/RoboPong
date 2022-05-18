function varargout = DobotGUI(varargin)
% DobotGUI MATLAB code for DobotGUI.fig
%      DobotGUI, by itself, creates a new DobotGUI or raises the existing
%      singleton*.
%
%      H = DobotGUI returns the handle to a new DobotGUI or the handle to
%      the existing singleton*.
%
%      DobotGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DobotGUI.M with the given input arguments.
%
%      DobotGUI('Property','Value',...) creates a new DobotGUI or raises the
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

% Last Modified by GUIDE v2.5 18-May-2022 00:52:18

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

% Executes just before DobotGUI is made visible.
function DobotGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to DobotGUI (see VARARGIN)

% Choose default command line output for DobotGUI
handles.output = hObject;

% Update handles structure

% UIWAIT makes DobotGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);

robot =  Dobot;
handles.robot = robot;

handles.btnEstop_count = 0;

hold on
% Place in the Floor
surf([-4,-4 ; 4,4], [-4,4 ; -4,4], [0,0 ; 0,0],'CData',imread('Marble.jpg'),'FaceColor','texturemap');                  
surf([-2,-2;2,2],[-1.5,1.5;-1.5,1.5],[0,0;0,0],'CData',imread('Carpet.jpg'),'FaceColor','texturemap');

%Placement of Table (from https://free3d.com/3d-model/straight-leg-coffee-tablewhite-v1--558417.html)
PlaceObject('Table.ply',[0,0,0]);

% Placement of Fence (from https://free3d.com/3d-model/fence-43609.html)
PlaceObject('Fence1.ply', [-4, 2,1.7]);
PlaceObject('Fence1.ply', [-4,-2,1.7]);
PlaceObject('Fence1.ply', [ 4, 2,1.7]);
PlaceObject('Fence1.ply', [ 4,-2,1.7]);
PlaceObject('Fence2.ply', [ 2,-4,1.7]);
PlaceObject('Fence2.ply', [-2,-4,1.7]);

% Placement of Emergency Stop Button (from https://free3d.com/3d-model/emergency-stop-button-813870.html)
PlaceObject('Emergency_Stop.ply',[-3,-3.8,1]);

% Placement of Fire Extinguisher (from https://free3d.com/3d-model/-fire-extinguisher-v3--639064.html)
PlaceObject('Fire_Extinguisher.ply',[-3.8,3.25,0.55]);

% Placement of Manager (from https://www.cgtrader.com/items/889520/.2download-page)
PlaceObject('Manager.ply', [3, 3, 0]);
PlaceObject('Man.ply', [-2.2, 0, 0]);

% Placement of Trash Can (from https://free3d.com/3d-model/rubbish-bin-83371.html)
PlaceObject('Bin.ply',[-3.8,2.5,0]);

% Placement of Sink (https://www.cgtrader.com/items/948227/download-page)
PlaceObject('Sink.ply', [3, -3.8, 0]);

% Placement of Storage Container (from https://free3d.com/3d-model/storage-container-v2--782422.html)
PlaceObject('Storage.ply',[3.5,0,0]);
PlaceObject('Storage.ply',[3.5,-1.25,0]);

% Placement of Bar (https://www.cgtrader.com/items/173511/download-page)
PlaceObject('Bar.ply',[2.5,2.25,0]);

% Placement of Stool (from https://free3d.com/3d-model/wood-stool-303532.html)
PlaceObject('Stool.ply',[0,-3.3,0]);
PlaceObject('Stool.ply',[0.75,-3.3,0]);
PlaceObject('Stool.ply',[1.65,-3.3,0]);
PlaceObject('Stool.ply',[-3.65,1.5,0]);
PlaceObject('Stool.ply',[-3.25,1.1,0]);
PlaceObject('Stool.ply',[-3.5,-2.2,0]);
PlaceObject('Stool.ply',[1,3.3,0]);
PlaceObject('Stool.ply',[1,2.5,0]);
PlaceObject('Stool.ply',[1,1.8,0]);

guidata(hObject,handles);

% Outputs from this function are returned to the command line.
function varargout = DobotGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Get default command line output from handles structure
varargout{1} = handles.output;

%THETA 1 NUMBER BOX ------------------------------------------------------------------------------------------------------------------------------------------------------------

function theta1Num_Callback(hObject, eventdata, handles)
% hObject    handle to theta1Num (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of theta1Num as text
%        str2double(get(hObject,'String')) returns contents of theta1Num as a double
% Executes during object creation, after setting all properties.
function theta1Num_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta1Num (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


%THETA 2 NUMBER BOX------------------------------------------------------------------------------------------------------------------------------------------------------------

function theta2Num_Callback(hObject, eventdata, handles)
% hObject    handle to theta2Num (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of theta2Num as text
%        str2double(get(hObject,'String')) returns contents of theta2Num as a double
% Executes during object creation, after setting all properties.
function theta2Num_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta2Num (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%THETA 3 NUMBER BOX------------------------------------------------------------------------------------------------------------------------------------------------------------

function theta3Num_Callback(hObject, eventdata, handles)
% hObject    handle to theta3Num (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of theta3Num as text
%        str2double(get(hObject,'String')) returns contents of theta3Num as a double
%Executes during object creation, after setting all properties.
function theta3Num_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta3Num (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%THETA 4 NUMBER BOX---------------------------------------------------------------------------------------------------------------------------

function theta4Num_Callback(hObject, eventdata, handles)
% hObject    handle to theta4Num (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of theta4Num as text
%        str2double(get(hObject,'String')) returns contents of theta4Num as a double
% Executes during object creation, after setting all properties.
function theta4Num_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta4Num (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%THETA 1 SLIDER---------------------------------------------------------------------------------------------------------------------------

function theta1Slider_Callback(hObject, eventdata, handles)
% hObject    handle to theta1Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% HInts: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    val = round(get(hObject, 'Value'));
    L = handles.robot.model.getpos();
    L(1) = deg2rad(val);
    handles.robot.model.animate(L);
    
        
% --- Executes during object creation, after setting all properties.
function theta1Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta1Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    set(hObject, 'Max', 135, 'Min', -135);
    set(hObject, 'SliderStep', [1/269 1]);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

%THETA 2 SLIDER-----------------------------------------------------------------------------------------------------------------------------------------------------------------------

% Executes on slider movement.
function theta2Slider_Callback(hObject, eventdata, handles)
% hObject    handle to theta2Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    val = round(get(hObject, 'Value'));
    L = handles.robot.model.getpos();
    L(2) = deg2rad(val);
    handles.robot.model.animate(L);

% Executes during object creation, after setting all properties.
function theta2Slider_CreateFcn(hObject, eventdata, handles)
    set(hObject, 'Max', 80, 'Min', 0);
    set(hObject, 'SliderStep', [1/79 1]);
% hObject    handle to theta2Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

%THETA 3 SLIDER-----------------------------------------------------------------------------------------------------------------------------------------------------------------------

% Executes on slider movement.
function theta3Slider_Callback(hObject, eventdata, handles)
% hObject    handle to theta3Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    val = round(get(hObject, 'Value'));
    L = handles.robot.model.getpos();
    L(3) = deg2rad(val);
    handles.robot.model.animate(L);

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

%Executes during object creation, after setting all properties.
function theta3Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta3Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    set(hObject, 'Max', 95, 'Min', -10);
    set(hObject, 'SliderStep', [1/104 1]);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

%THETA 4 SLIDER-----------------------------------------------------------------------------------------------------------------------------------------------------------------------

% Executes on slider movement.
function theta4Slider_Callback(hObject, eventdata, handles)
% hObject    handle to theta4Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    val = round(get(hObject, 'Value'));
    L = handles.robot.model.getpos();
    L(4) = deg2rad(val); 
    handles.robot.model.animate(L);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function theta4Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta4Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    set(hObject, 'Max', 180, 'Min', -180);
    set(hObject, 'SliderStep', [1/104 1]);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

%FORWARD KINEMATICS---------------------------------------------------------------------------------------------------------------------------
% Executes on button press in btnForward.
function btnForward_Callback(hObject, eventdata, handles)
% hObject    handle to btnForward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
th1 = str2double(handles.theta1Num.String)*pi/180;
th2 = str2double(handles.theta2Num.String)*pi/180;
th3 = str2double(handles.theta3Num.String)*pi/180;
th4 = str2double(handles.theta4Num.String)*pi/180;

handles.model.plot([th1 th1 th3 th4]);

T = handles.model.fkine([th1 th2 th3 th4]);


%END EFFECTOR X---------------------------------------------------------------------------------------------------------------------------

function posX_Callback(hObject, eventdata, handles)
% hObject    handle to posX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of posX as text
%        str2double(get(hObject,'String')) returns contents of posX as a double
% Executes during object creation, after setting all properties.
function posX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%END EFFECTOR Y---------------------------------------------------------------------------------------------------------------------------

function posY_Callback(hObject, eventdata, handles)
% hObject    handle to posY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of posY as text
%        str2double(get(hObject,'String')) returns contents of posY as a double
% Executes during object creation, after setting all properties.
function posY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%END EFFECTOR Z---------------------------------------------------------------------------------------------------------------------------

function posZ_Callback(hObject, eventdata, handles)
% hObject    handle to posZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of posZ as text
%        str2double(get(hObject,'String')) returns contents of posZ as a double
% --- Executes during object creation, after setting all properties.
function posZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%INVERSE KINEMATICS---------------------------------------------------------------------------------------------------------------------------

% Executes on button press in btnInverse.
function btnInverse_Callback(hObject, eventdata, handles)
% hObject    handle to btnInverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% --- Executes on slider movement.

%E-STOP BUTTON-----------------------------------------------------------------------------------------------------------------------------------------------------------------------

%Executes on button press in btnEstop.
function btnEstop_Callback(hObject, eventdata, handles)
% hObject    handle to btnEstop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.btnEstop_count == 0
    handles.btnEstop_count = 1;
    guidata(hObject, handles);
    handles.stop = 1;
    disp('Estop Activated')
    uiwait();
    handles.btnEstop_count =0;
    guidata(hObject, handles);
end

if handles.btnEstop_count == 1
    handles.btnEstop_count =2;
    disp('Estop Released')
    guidata(hObject, handles);
end

%DISPLAY ACTIVE/INACTIVE STATE-----------------------------------------------------------------------------------------------------------------------------------------------------------------------

function check_Callback(hObject, eventdata, handles)
% hObject    handle to check (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of check as text
%        str2double(get(hObject,'String')) returns contents of check as a double
%Executes during object creation, after setting all properties.
if handles.btnEstop_count == 0 || handles.btnEstop_count == 2
    set(handles.check, 'Estop Inactive');
    guidata(hObject, handles);
end
if handles.btnEstop_count == 1
    set(handles.check, 'Estop Active');
    guidata(hObject, handles);
end
    

function check_CreateFcn(hObject, eventdata, handles)
% hObject    handle to check (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%RESUME AFTER E-STOP PRESSED-----------------------------------------------------------------------------------------------------------------------------------------------------------------------

% --- Executes on button press in btnResume.
function btnResume_Callback(hObject, eventdata, handles)
% hObject    handle to btnResume (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% GetDobotRobot
if handles.btnEstop_count==2
    disp('Resuming')
    uiresume();
else
    disp('Estop is Activated')
end

%RED PING PONG BALL SELECTION-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
% --- Executes on button press in redBall.
function redBall_Callback(hObject, eventdata, handles)
% hObject    handle to redBall (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.btnEstop_count == 0
    Red(handles.robot)
    disp('Catching Red Ball')
else
    disp("Estop is Activated")
end
% Hint: get(hObject,'Value') returns toggle state of redBall

%YELLOW PING PONG BALL SELECTION-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
% --- Executes on button press in yellowBall.
function yellowBall_Callback(hObject, eventdata, handles)
% hObject    handle to yellowBall (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.btnEstop_count == 0
    Yellow(handles.robot)
    disp('Catching Yellow Ball')
else
    disp("Estop is Activated")
end
% Hint: get(hObject,'Value') returns toggle state of yellowBall

%RED PING PONG BALL COLLISION SELECTION-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
% --- Executes on button press in redBallC.
function redBallC_Callback(hObject, eventdata, handles)
% hObject    handle to redBallC (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.btnEstop_count == 0
    RedCollision(handles.robot)
    disp('Catching Red Ball')
else
    disp("Estop is Activated")
end

%YELLOW PING PONG BALL COLLISION SELECTION-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
% --- Executes on button press in yellowBallC.
function yellowBallC_Callback(hObject, eventdata, handles)
% hObject    handle to yellowBallC (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.btnEstop_count == 0
    YellowCollision(handles.robot)
    disp('Catching Yellow Ball')
else
    disp("Estop is Activated")
end