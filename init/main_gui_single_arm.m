%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     FANUC LRMate200iD/7L Dual Robot Experimentor
%       Main GUI for Single Arm Platform
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Created by Wenjie Chen, FANUC Corporation, 2016/07/11
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function varargout = main_gui_single_arm(varargin)
% MAIN_GUI_SINGLE_ARM MATLAB code for main_gui_single_arm.fig
%      MAIN_GUI_SINGLE_ARM, by itself, creates a new MAIN_GUI_SINGLE_ARM or raises the existing
%      singleton*.
%
%      H = MAIN_GUI_SINGLE_ARM returns the handle to a new MAIN_GUI_SINGLE_ARM or the handle to
%      the existing singleton*.
%
%      MAIN_GUI_SINGLE_ARM('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MAIN_GUI_SINGLE_ARM.M with the given input arguments.
%
%      MAIN_GUI_SINGLE_ARM('Property','Value',...) creates a new MAIN_GUI_SINGLE_ARM or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before main_gui_single_arm_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to main_gui_single_arm_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help main_gui_single_arm

% Last Modified by GUIDE v2.5 30-Aug-2016 22:12:06

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @main_gui_single_arm_OpeningFcn, ...
    'gui_OutputFcn',  @main_gui_single_arm_OutputFcn, ...
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


% --- Executes just before main_gui_single_arm is made visible.
function main_gui_single_arm_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to main_gui_single_arm (see VARARGIN)

% Link base space variables to handles
handles.si = evalin('base', 'si');

% Choose default command line output for main_gui_single_arm
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes main_gui_single_arm wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = main_gui_single_arm_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in Stop_App.
function Stop_App_Callback(hObject, eventdata, handles)

brake_on_off(handles.si.ParamSgnID, 'on');
tg_start_stop('stop');


% --- Executes on button press in LTT_Replay_button.
function LTT_Replay_button_Callback(hObject, eventdata, handles)

LTT_Motion_Planning(handles.si);


% --- Executes on button press in LTT_Teaching.
function LTT_Teaching_Callback(hObject, eventdata, handles)

LTT_Teaching(handles.si, hObject);


% --- Executes on button press in Mouse_Teaching.
function Mouse_Teaching_Callback(hObject, eventdata, handles)

Mouse_Teaching(handles.si);


% --- Executes on button press in Test_Picking.
function Test_Picking_Callback(hObject, eventdata, handles)

Test_Picking(handles.si);


% --- Executes on button press in Cam_Open.
function Cam_Open_Callback(hObject, eventdata, handles)

handles.si.k2 = cam_open(handles.si.camera_type);


% --- Executes on button press in Cam_Close.
function Cam_Close_Callback(hObject, eventdata, handles)

cam_close(handles.si.camera_type, handles.si.k2);


% --- Executes on button press in Init_Pos_Back.
function Init_Pos_Back_Callback(hObject, eventdata, handles)

init_pos_back(handles.si);
