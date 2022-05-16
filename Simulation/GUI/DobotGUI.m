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

% Last Modified by GUIDE v2.5 16-May-2022 23:25:11

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
guidata(hObject, handles);

% UIWAIT makes DobotGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);

axes(handles.axes1);

L(1) = Link('d', 0.09, 'a', 0, 'alpha', pi/2, 'offset', 0, 'qlim', [-3*pi/4,3*pi/4]);
L(2) = Link('d', 0, 'a', -0.152, 'alpha', 0,'offset', deg2rad(-32.5), 'qlim', [0,17*pi/36]);
L(3) = Link('d', 0, 'a', -0.147, 'alpha', 0, 'offset', deg2rad(62.25), 'qlim', [-pi/18,19*pi/36]);
L(4) = Link('d', 0, 'a', 0.02, 'alpha', 0, 'offset', deg2rad(-30), 'qlim', [-pi/2,pi/2]);
model = SerialLink(L, 'name', 'Dobot');

model.base = transl(0.55,0,1.145);

% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available

for linkIndex = 0:model.n
    [faceData, vertexData, plyData{linkIndex + 1}] = plyread(['DobotMagicianLink', num2str(linkIndex), '.ply'], 'tri'); %#ok<AGROW>
    model.faces{linkIndex + 1} = faceData;
    model.points{linkIndex + 1} = vertexData;
end

% Display robot
workspace = [-5 5 -5 2 -0.3 2]; 
model.plot3d(zeros(1, model.n), 'noarrow', 'workspace', workspace);
if isempty(findobj(get(gca, 'Children'), 'Type', 'Light'))
    camlight
end
model.delay = 0;

% Try to correctly colour the arm (if colours are in ply file data)
for linkIndex = 0:model.n
    handles = findobj('Tag', model.name);
    h = get(handles, 'UserData');
    try
        h.link(linkIndex + 1).Children.FaceVertexCData = [plyData{linkIndex + 1}.vertex.red ...
                                                        , plyData{linkIndex + 1}.vertex.green ...
                                                        , plyData{linkIndex + 1}.vertex.blue] / 255;
        h.link(linkIndex + 1).Children.FaceColor = 'interp';
    catch ME_1
        disp(ME_1);
        continue;
    end
end


% Building 3D Environment

hold on

% Place in Concrete Floor
surf([-6,-6 ; 10,10], [-6,6 ; -6,6], [0,0 ; 0,0],'CData',imread('Concrete.jpg'),'FaceColor','texturemap');            % From UTS Canvas
surf([-4,-4 ; 8,8], [-4,4 ; -4,4], [0,0 ; 0,0],'CData',imread('Tile.jpg'),'FaceColor','texturemap'); 

% Place in Hazard Tape (from https://nationalsafetysigns.com.au/safety-signs/reflective-tape-sticker-hazard-tape-v2628/)
surf([-2,-2;2,2],[-1.5,1.5;-1.5,1.5],[0,0;0,0],'CData',imread('Tape.jpg'),'FaceColor','texturemap');

%Placement of Table (from https://free3d.com/3d-model/straight-leg-coffee-tablewhite-v1--558417.html)
PlaceObject('Table.ply',[0,0,0]);

% Placement of Fence (from https://free3d.com/3d-model/fence-43609.html)
PlaceObject('Fence1.ply', [-4, 2,1.7]);
PlaceObject('Fence1.ply', [-4,-2,1.7]);
PlaceObject('Fence1.ply', [ 8, 2,1.7]);
PlaceObject('Fence1.ply', [ 8,-2,1.7]);
PlaceObject('Fence2.ply', [ 2,-4,1.7]);
PlaceObject('Fence2.ply', [-2,-4,1.7]);
PlaceObject('Fence2.ply', [ 6,-4,1.7]);
PlaceObject('Fence2.ply', [ 6, 4,1.7]);
PlaceObject('Fence2.ply', [-2, 4,1.7]);

% Placement of Emergency Stop Button (from https://free3d.com/3d-model/emergency-stop-button-813870.html)
PlaceObject('Emergency_Stop.ply',[3,-3.8,1]);
PlaceObject('Emergency_Stop.ply',[-3,-3.8,1]);

% Placement of Fire Extinguisher (from https://free3d.com/3d-model/-fire-extinguisher-v3--639064.html)
PlaceObject('Fire_Extinguisher.ply',[-3.8,3.25,0.55]);

% Placement of Worker (from https://www.cgtrader.com/items/889520/download-page)
PlaceObject('Worker.ply',[-1,-1.75,0]);
PlaceObject('Worker2.ply', [2.2, 0, 0]);

% Placement of Trash Can (from https://free3d.com/3d-model/rubbish-bin-83371.html)
PlaceObject('Bin.ply',[-3.8,2.5,0]);

% Placement of Plants (from ???)
PlaceObject('Plants.ply', [6, 4.5, 0])
PlaceObject('Plants.ply', [-2, 4.5, 0])

% Placement of Sink (https://www.cgtrader.com/items/948227/download-page)
PlaceObject('Sink.ply', [6, -3.8, 0])

% Placement of Storage Container (from https://free3d.com/3d-model/storage-container-v2--782422.html)
PlaceObject('Storage.ply',[-3.5,0,0]);
PlaceObject('Storage.ply',[-3.5,-1.25,0]);
PlaceObject('Storage.ply',[7.5,0,0]);
PlaceObject('Storage.ply',[7.5,-1.25,0]);
PlaceObject('Storage.ply',[7.5,1.25,0]);

% Placement of Stool (from https://free3d.com/3d-model/wood-stool-303532.html)
PlaceObject('Stool.ply',[0,-3.3,0]);
PlaceObject('Stool.ply',[0.75,-3.3,0]);
PlaceObject('Stool.ply',[1.65,-3.3,0]);
PlaceObject('Stool.ply',[-3.65,1.5,0]);
PlaceObject('Stool.ply',[-3.25,1.1,0]);
PlaceObject('Stool.ply',[-3.5,-2.2,0]);

data = guidata(hObject);
data.model = model;
guidata(hObject,data);

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
    L = handles.model.getpos();
    L(1) = deg2rad(val);
    handles.model.animate(L);
        
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
    L = handles.model.getpos();
    L(2) = deg2rad(val);
    handles.model.animate(L);

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
    L = handles.model.getpos();
    L(3) = deg2rad(val);
    handles.model.animate(L);

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
    L = handles.model.getpos();
    L(4) = deg2rad(val); 
    handles.model.animate(L);
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

%DISPLAY ACTIVE/INACTIVE STATE-----------------------------------------------------------------------------------------------------------------------------------------------------------------------

function check_Callback(hObject, eventdata, handles)
% hObject    handle to check (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of check as text
%        str2double(get(hObject,'String')) returns contents of check as a double
%Executes during object creation, after setting all properties.
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


% --- Executes on button press in redBall.
function redBall_Callback(hObject, eventdata, handles)
% hObject    handle to redBall (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of redBall


% --- Executes on button press in yellowBall.
function yellowBall_Callback(hObject, eventdata, handles)
% hObject    handle to yellowBall (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of yellowBall
