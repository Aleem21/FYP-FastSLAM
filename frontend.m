function varargout = frontend(varargin)
% FRONTEND MATLAB code for frontend.fig
%      FRONTEND, by itself, creates a new FRONTEND or raises the existing
%      singleton*.
%
%      H = FRONTEND returns the handle to a new FRONTEND or the handle to
%      the existing singleton*.
%
%      FRONTEND('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in FRONTEND.M with the given input arguments.
%
%      FRONTEND('Property','Value',...) creates a new FRONTEND or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before frontend_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to frontend_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help frontend

% Last Modified by GUIDE v2.5 06-Mar-2016 14:48:45

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @frontend_OpeningFcn, ...
                   'gui_OutputFcn',  @frontend_OutputFcn, ...
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


% --- Executes just before frontend is made visible.
function frontend_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to frontend (see VARARGIN)

% Choose default command line output for frontend
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes frontend wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = frontend_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in Mapping.
function Mapping_Callback(hObject, eventdata, handles)
% hObject    handle to Mapping (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Mapping


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fastslam2r_sim(handles.landmarks, handles.waypoints);


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[file_name, path_name] = uigetfile({'*.mp4','*.mkv'},'/Users/searcherliu/Desktop/FYP/Lab/myfastslam/UFastSLAM_Fin');
handles.name=strcat(path_name,file_name);
set(handles.edit1,'String', handles.name);
guidata(hObject,handles);


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pushbutton3.
function pushbutton3_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double




% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fastslam2rU_sim(handles.landmarks, handles.waypoints);


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fastslam2rUW_sim(handles.landmarks, handles.waypointsp);


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
h = waitbar(0,'Calibrating...');
calibrate;
close(h);
input_path = handles.name;
n = waitbar(0,'Creating Maps...');
[lm, wp] = visu_mapping(input_path);
handles.landmarks = lm;handles.waypoints = wp;
handles.waypoints = [0 6.2343   12.6574   25.0000   30.7935   17.8212    7.2418    0.1152   -4.2627  -17.4433  -30.0378  -28.6524 -19.9622   -5.4147    0.3456;0  -14.8423  -24.6753  -18.7384    4.4527   17.4397   14.2857   -0.4386   -7.4561  -25.7885  -12.0594    7.4212 20.7792   10.3801   -0.1462];
handles.landmarks = [-14.8618   -1.4977   21.7742   19.2396  -22.6071   33.1864   25.5038   41.1209   21.5995  -19.2065  -34.8237  -36.0831 -30.6675  -21.5995   -6.7380;5.9942   -9.7953   -8.6257   15.9357   -1.1132   18.1818    3.7106  -10.0186  -37.4768   29.4991   16.1410   -3.8961 -23.3766  -34.3228  -27.4583];
guidata(hObject,handles);
close(n);



% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fastslam2rUT_sim(handles.landmarks, handles.waypoints);


% --- Executes during object creation, after setting all properties.
function axes10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes10
k = imread('logo2.png');imshow(k);


% --- Executes during object creation, after setting all properties.
function axes11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes11
n = imread('Liverpool.jpg');imshow(n);
