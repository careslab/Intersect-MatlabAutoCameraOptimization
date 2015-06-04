function varargout = intersect(varargin)
% OPTIMIZEVIEWANGLE MATLAB code for optimizeviewangle.fig
%      OPTIMIZEVIEWANGLE, by itself, creates a new OPTIMIZEVIEWANGLE or raises the existing
%      singleton*.
%
%      H = OPTIMIZEVIEWANGLE returns the handle to a new OPTIMIZEVIEWANGLE or the handle to
%      the existing singleton*.
%
%      OPTIMIZEVIEWANGLE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in OPTIMIZEVIEWANGLE.M with the given input arguments.
%
%      OPTIMIZEVIEWANGLE('Property','Value',...) creates a new OPTIMIZEVIEWANGLE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before intersect_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to intersect_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help optimizeviewangle

% Last Modified by GUIDE v2.5 01-Apr-2015 13:40:30

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @intersect_OpeningFcn, ...
                   'gui_OutputFcn',  @intersect_OutputFcn, ...
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


% --- Executes just before optimizeviewangle is made visible.
function intersect_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to optimizeviewangle (see VARARGIN)
% This is a very quick user interfact to the PointOfView class.
% It is used to just test the class and visualize it's outputs.
% Choose default command line output for optimizeviewangle
handles.output = hObject;
handles.pv = PointOfView();

% model_data is used in the GUI.
handles.model_data = [ 2 3 3; 3 3 3; 1 0 0; 0 0 0; 35 0 0; 1.0 0 0 ];
                     % view vector, spherepos, Sphere size, Pyramid, FOV, Depth of field
set (handles.ModelParemeters, 'Data', handles.model_data);
 
handles.pv.field_of_view = handles.model_data(5,1);
handles.pv.flength = 5;  % flength is not allowed to be change...for now.
handles.pv.depth_of_field = handles.model_data(6,1);
handles.pv.pyramid_lookat = handles.model_data(1,:);
handles.pv.pyramid_pos = handles.model_data(4,:);
handles.tess = 10;

%Draw the initial stuff.
axes (handles.plotwindow);

rotate3d on;
set(handles.figure1, 'Toolbar', 'figure');
axis manual;

%place some objects AddObject (x, y, z, [pos], size, importance)
[x y z] = sphere (handles.tess);
handles.pv = handles.pv.AddObject(x, y, z, [ 3.0 3.0 3], 0.5, 1);
handles.pv = handles.pv.AddObject(x, y, z, [ 1.0 4.0 3], 0.5, .25);
%handles.pv = handles.pv.AddObject(x, y, z, [ 1 2.75 2.75], 0.5, .1);
%handles.pv = handles.pv.AddObject(x, y, z, [ 2 3 2], 0.5, .1);


handles.pv.DrawObjects(handles.plotwindow);
hold on;
%draw a pyramid
% Create the two pyramids from the default viewing pyramid.
handles.pv.pyramid   = handles.pv.CreatePyramid(handles.pv.field_of_view,handles.pv.pyramid_pos,handles.pv.pyramid_lookat,handles.pv.flength,0);
handles.pv.pyramid_dof  = handles.pv.CreatePyramid(handles.pv.field_of_view,handles.pv.pyramid_pos,handles.pv.pyramid_lookat,(handles.pv.flength - handles.pv.depth_of_field),0);
axes (handles.plotwindow);
handles.pv.DrawViewPyramid(handles.plotwindow);



%Check each point for intersection.
[percentage, in,  out] = handles.pv.CollisionPercentage();
hold on;
handles.pv.DrawInOutHullPoints(handles.plotwindow, in ,out);

%post the percentage coverage.
set (handles.Percentage, 'String' , num2str(percentage));


% Update handles structure
guidata(hObject, handles);

% UIWAIT makes optimizeviewangle wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = intersect_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in UpdateModels.
function UpdateModels_Callback(hObject, eventdata, handles)
% hObject    handle to UpdateModels (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%draw a pyramid
hold off;
axis manual;
alpha ('clear');
axis  ('vis3d');
axis ('manual');

%Make sure to update the new pyramids...if they changed.
handles.pv.pyramid   = handles.pv.CreatePyramid(handles.pv.field_of_view,handles.pv.pyramid_pos,handles.pv.pyramid_lookat,handles.pv.flength,0);
handles.pv.pyramid_dof  = handles.pv.CreatePyramid(handles.pv.field_of_view,handles.pv.pyramid_pos,handles.pv.pyramid_lookat,(handles.pv.flength - handles.pv.depth_of_field),0);


%Draw the pyramid
handles.pv.DrawViewPyramid(handles.plotwindow);
hold on;    
%draw the objects
handles.pv.DrawObjects(handles.plotwindow);
hold on;
%Check each point for intersection.
[percentage, in, out] = handles.pv.CollisionPercentage();
handles.pv.DrawInOutHullPoints(handles.plotwindow, in ,out);

%post the percentage coverage.
set (handles.Percentage, 'String' , num2str(percentage));



% --- Executes when entered data in editable cell(s) in ModelParemeters.
function ModelParemeters_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to ModelParemeters (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)
handles.model_data = get(hObject, 'Data');

handles.pv.field_of_view = handles.model_data(5,1);
handles.pv.flength = 5;
handles.pv.depth_of_field = handles.model_data(6,1);
handles.pv.pyramid_lookat = handles.model_data(1,:);
handles.pv.pyramid_pos = handles.model_data(4,:);

%Make sure to update the new pyramids...if they changed.
handles.pv.pyramid   = handles.pv.CreatePyramid(handles.pv.field_of_view,handles.pv.pyramid_pos,handles.pv.pyramid_lookat,handles.pv.flength,0);
handles.pv.pyramid_dof  = handles.pv.CreatePyramid(handles.pv.field_of_view,handles.pv.pyramid_pos,handles.pv.pyramid_lookat,(handles.pv.flength - handles.pv.depth_of_field),0);


% Update handles structure
guidata(hObject, handles);

UpdateModels_Callback(hObject, eventdata, handles);




% --- Executes on button press in OptimizeViewAngle.
function OptimizeViewAngle_Callback(hObject, eventdata, handles)
% hObject    handle to OptimizeViewAngle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Sphere(1) = struct ('x',handles.sx, 'y', handles.sy, 'z', handles.sz, 'importance', 1);
% Sphere(2) = struct ('x',handles.sx2, 'y', handles.sy2, 'z', handles.sz2, 'importance', 1);

%[e] =OptLookat(Sphere, handles.field_of_view, handles.pyramid_lookat, handles.flength, handles.depth_of_field)
%OptLookat(Sphere, handles.field_of_view, handles.pyramid_lookat,handles.pyramid_pos, handles.flength, handles.depth_of_field);
%pyramid_opt = [handles.pyramid_lookat handles.pyramid_pos];
UpdateModels_Callback(hObject, eventdata, handles);

% Optimize the view look at point (angles)
handles.pv =  handles.pv.OptimizeLookAt(); %optimizes the look at point.

%update the table
handles.model_data(1,:) = handles.pv.pyramid_lookat;


 set (handles.ModelParemeters, 'Data', handles.model_data);
 
 % Update handles structure
guidata(hObject, handles);
UpdateModels_Callback(hObject, eventdata, handles);
                     



% --- Executes on button press in OptimizeViewPos.
function OptimizeViewPos_Callback(hObject, eventdata, handles)
% hObject    handle to OptimizeViewPos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Now optimize the pos (or somewhat the zoom value).

 
handles.pv = handles.pv.OptimizeLookFrom(); % optimizes the look from point.

handles.model_data(4,:) = handles.pv.pyramid_pos;
    
%handles.model_data(1,:) = handles.pyramid_lookat;
 set (handles.ModelParemeters, 'Data', handles.model_data);
 
 % Update handles structure
guidata(hObject, handles);
UpdateModels_Callback(hObject, eventdata, handles);


% --- Executes on slider movement.
function ObjectNum_Callback(hObject, eventdata, handles)
% hObject    handle to ObjectNum (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function ObjectNum_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ObjectNum (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in AddObject.
function AddObject_Callback(hObject, eventdata, handles)
% hObject    handle to AddObject (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[x y z] = sphere (handles.tess);

s = inputdlg ('Importance?', 'Importance?');
imp = str2num (s{1});


handles.pv = handles.pv.AddObject (x, y, z, handles.model_data(2,:), 1, imp);
hold off;
handles.pv.DrawViewPyramid(handles.plotwindow);
hold on;
handles.pv.DrawObjects(handles.plotwindow);
 % Update handles structure
guidata(hObject, handles);


% --- Executes on button press in ChangeObjectPos.
function ChangeObjectPos_Callback(hObject, eventdata, handles)
% hObject    handle to ChangeObjectPos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
s = inputdlg ('which object num', 'objec num:');
i = str2num (s{1});
if ((i > handles.pv.num_objects) || (i < 0))
    disp (' object num out of range');
    return;
end
s = inputdlg ('Position x y z', 'Object pos:');
pos = str2num (s{1});

handles.pv = handles.pv.Change_object_pos(i, pos);

% Update handles structure
guidata(hObject, handles);
UpdateModels_Callback(hObject, eventdata, handles);


% --- Executes on button press in Exitpv.
function Exitpv_Callback(hObject, eventdata, handles)
% hObject    handle to Exitpv (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close (handles.figure1);
clear all
clear classes


% --- Executes on button press in OptimizeZoom.
function OptimizeZoom_Callback(hObject, eventdata, handles)
% hObject    handle to OptimizeZoom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.pv = handles.pv.OptimizeZoom(); % optimizes the look from point.

handles.model_data(4,:) = handles.pv.pyramid_pos;
    
%handles.model_data(1,:) = handles.pyramid_lookat;
 set (handles.ModelParemeters, 'Data', handles.model_data);
 
 % Update handles structure
guidata(hObject, handles);
UpdateModels_Callback(hObject, eventdata, handles);


% --- Executes on button press in OptimizeZoomAndLookat.
function OptimizeZoomAndLookat_Callback(hObject, eventdata, handles)
% hObject    handle to OptimizeZoomAndLookat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.pv = handles.pv.OptimizeZoomAndLookat(); % optimizes the look from point.

handles.model_data(4,:) = handles.pv.pyramid_pos;
handles.model_data(1,:) = handles.pv.pyramid_lookat;
 set (handles.ModelParemeters, 'Data', handles.model_data);
 
 % Update handles structure
guidata(hObject, handles);
UpdateModels_Callback(hObject, eventdata, handles);


% --- Executes on key press with focus on ChangeObjectPos and none of its controls.
function ChangeObjectPos_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to ChangeObjectPos (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)
