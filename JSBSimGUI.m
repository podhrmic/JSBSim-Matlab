function varargout = JSBSimGUI(varargin)
% JSBSIMGUI M-file for JSBSimGUI.fig
%      JSBSIMGUI, by itself, creates a new JSBSIMGUI or raises the existing
%      singleton*.
%
%      H = JSBSIMGUI returns the handle to a new JSBSIMGUI or the handle to
%      the existing singleton*.
%
%      JSBSIMGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in JSBSIMGUI.M with the given input arguments.
%
%      JSBSIMGUI('Property','Value',...) creates a new JSBSIMGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before JSBSimGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to JSBSimGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help JSBSimGUI

% Last Modified by GUIDE v2.5 10-Jan-2010 06:00:11

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @JSBSimGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @JSBSimGUI_OutputFcn, ...
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


% --- Executes just before JSBSimGUI is made visible.
function JSBSimGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to JSBSimGUI (see VARARGIN)

%file_open(handles)
% Choose default command line output for JSBSimGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes JSBSimGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = JSBSimGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% function file_open(handles)
%     open JSBSim_model_data.m
    
% Takes name of Simulink model that contains the JSBSimSFunction
function simulink_model_Callback(hObject, eventdata, handles)
% hObject    handle to simulink_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.NameOfModel = get(hObject,'String');
    guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of simulink_model as text
%        str2double(get(hObject,'String')) returns contents of simulink_model as a double


% --- Executes on button press in open_model.
% function open_model_Callback(hObject, eventdata, handles)
% hObject    handle to open_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function simulink_model_CreateFcn(hObject, eventdata, handles)
% hObject    handle to simulink_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.NameOfModel = get(hObject,'String');
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in load_model.
function load_model_Callback(hObject, eventdata, handles)
% hObject    handle to load_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    open_system(handles.NameOfModel); %open the simulink model
    handles.ModelPath = strcat(handles.NameOfModel, '/JSBSim S-Function');
    
    %******************Get the JSBSim SFunction parameters
    handles.NameOfAircraft = get_param(handles.ModelPath,'ac_name');
    handles.ic_states = get_param(handles.ModelPath,'IC_states');
    handles.ic_controls = get_param(handles.ModelPath,'IC_controls');                   
    handles.ic_delta = get_param(handles.ModelPath,'IC_delta_t');   
    handles.NameOfVerbosity = get_param(handles.ModelPath,'IC_verbosity');
    handles.ic_multiplier = get_param(handles.ModelPath,'IC_sim_multiplier');
%     handles.ic_int_override = get_param(handles.ModelPath,'IC_int_override');
    %i = 0;
    
%     while true
%         [string_states, handles.ic_states] = strtok(handles.ic_states, '[]');
%         if isempty(string_states),  break;  end
%             disp(sprintf('%s', string_states))          
%     end
        
    for k = 1:14
        [str, handles.ic_states] = strtok(handles.ic_states);
            switch k
                case 2
                    disp(sprintf('%s', str))
                    handles.u_value =  str2double(str);%save new value as IC parameter
                    set(handles.IC_u,'Value',handles.u_value);%set slider to new value
                    set(handles.IC_u_value, 'String', str);
                case 3
                    handles.v_value =  str2double(str);%save new value as IC parameter
                    set(handles.IC_v,'Value',handles.v_value);%set slider to new value
                    set(handles.IC_v_value, 'String', str);
                case 4
                    handles.w_value =  str2double(str);%save new value as IC parameter
                    set(handles.IC_w,'Value',handles.w_value);%set slider to new value
                    set(handles.IC_w_value, 'String', str);
                case 5
                    handles.p_value =  str2double(str);%save new value as IC parameter
                    set(handles.IC_p,'Value',handles.p_value);%set slider to new value
                    set(handles.IC_p_value, 'String', str);
                case 6
                    handles.q_value =  str2double(str);%save new value as IC parameter
                    set(handles.IC_q,'Value',handles.q_value);%set slider to new value
                    set(handles.IC_q_value, 'String', str);
                case 7
                    handles.r_value =  str2double(str);%save new value as IC parameter
                    set(handles.IC_r,'Value',handles.r_value);%set slider to new value
                    set(handles.IC_r_value, 'String', str);
                case 8
                    handles.h_value =  str2double(str);%save new value as IC parameter
                    set(handles.IC_h,'Value',handles.h_value);%set slider to new value
                    set(handles.IC_h_value, 'String', str);
                case 9
                    handles.long_value =  str2double(str);%save new value as IC parameter
                    %set(handles.IC_long,'Value',handles.long_value);%set slider to new value
                    set(handles.IC_long, 'String', str);
                case 10
                    handles.lat_value =  str2double(str);%save new value as IC parameter
                    %set(handles.IC_lat,'Value',handles.lat_value);%set slider to new value
                    set(handles.IC_lat, 'String', str);
                case 11
                    handles.phi_value =  str2double(str);%save new value as IC parameter
                    set(handles.IC_phi,'Value',handles.phi_value);%set slider to new value
                    set(handles.IC_phi_value, 'String', str);
                case 12
                    handles.theta_value =  str2double(str);%save new value as IC parameter
                    set(handles.IC_theta,'Value',handles.theta_value);%set slider to new value
                    set(handles.IC_theta_value, 'String', str);
                case 13
                    handles.psi_value =  str2double(str);%save new value as IC parameter
                    set(handles.IC_psi,'Value',handles.psi_value);%set slider to new value
                    set(handles.IC_psi_value, 'String', str);
            end
            %i = i + 1;;
    end
    %j = 0;
    
%     while true
%         %strip off the brackets, put states in "controls_string"
%         [controls_string, handles.ic_controls] = strtok(handles.ic_controls, '[]');
%         if isempty(controls_string),  break;  end
%             disp(sprintf('%s', controls_string))
%             handles.ic_controls = controls_string;
%     end    
    
    for l = 1:10
        [str1, handles.ic_controls] = strtok(handles.ic_controls);
            switch l
                case 2
                    handles.throttle_value =  str2num(str1);%save new value as IC parameter
                    set(handles.IC_throttle,'Value',handles.throttle_value);%set slider to new value
                    set(handles.IC_throttle_value, 'String', str1);
                case 3
                    handles.aileron_value =  str2num(str1);%save new value as IC parameter
                    set(handles.IC_aileron,'Value',handles.aileron_value);%set slider to new value
                    set(handles.IC_aileron_value, 'String', str1);
                case 4
                    handles.elevator_value =  str2num(str1);%save new value as IC parameter
                    set(handles.IC_elevator,'Value',handles.elevator_value);%set slider to new value
                    set(handles.IC_elevator_value, 'String', str1);
                case 5
                    handles.rudder_value =  str2num(str1);%save new value as IC parameter
                    set(handles.IC_rudder,'Value',handles.rudder_value);%set slider to new value
                    set(handles.IC_rudder_value, 'String', str1);
                case 6
                    handles.mixture_value =  str2num(str1);%save new value as IC parameter
                    set(handles.IC_mixture,'Value',handles.mixture_value);%set slider to new value
                    set(handles.IC_mixture_value, 'String', str1);
                case 7
                    handles.set_run_value =  str2num(str1);%save new value as IC parameter
                    set(handles.IC_set_run,'Value',handles.set_run_value);%set radio button to new value
                    %set(handles.IC_set_run_value, 'String', str1);
                case 8
                    handles.flaps_value =  str2num(str1);%save new value as IC parameter
                    set(handles.IC_flaps,'Value',handles.flaps_value);%set slider to new value
                    set(handles.IC_flaps_value, 'String', str1);
                case 9
                    handles.gear_value =  str2num(str1);%save new value as IC parameter
                    set(handles.IC_gear,'Value',handles.gear_value);%set slider to new value
                    set(handles.IC_gear_value, 'String', str1);
                
            end
            %j = j + 1;
    end
   
    set(handles.AC_name,'String',handles.NameOfAircraft);
    set(handles.verbosity_name,'String',handles.NameOfVerbosity);
    set(handles.delta_T,'String',handles.ic_delta);
    handles.delta_t_value = str2double(handles.ic_delta);
    handles.multiplier_value = str2double(handles.ic_multiplier);
    set(handles.IC_multiple,'Value',handles.multiplier_value);%set slider to new value
    set(handles.IC_multiple_value,'String',handles.ic_multiplier);
%     handles.set_int_override = str2num(handles.ic_int_override);
%     set(handles.IC_int_override_value,'Value',handles.set_int_override);%set radio button to new value
    guidata(hObject,handles);
    
% --- Executes on selection change in AC_files.
function AC_files_Callback(hObject, eventdata, handles)
% hObject    handle to AC_files (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns AC_files contents as cell array
%        contents{get(hObject,'Value')} returns selected item from AC_files


% --- Executes during object creation, after setting all properties.
function AC_files_CreateFcn(hObject, eventdata, handles)
% hObject    handle to AC_files (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function AC_name_Callback(hObject, eventdata, handles)
% hObject    handle to AC_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.NameOfAircraft = get(hObject,'String');
    
    guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of AC_name as text
%        str2double(get(hObject,'String')) returns contents of AC_name as a double


% --- Executes during object creation, after setting all properties.
function AC_name_CreateFcn(hObject, eventdata, handles)
% hObject    handle to AC_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.AC_name = hObject;
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
    

% --- Executes on selection change in verbosity.
function verbosity_Callback(hObject, eventdata, handles)
% hObject    handle to verbosity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
   
    index = get(hObject,'Value');
    %contents = get(hObject,'String');
    new_verbosity = handles.verbosity_contents{index};
    set(handles.verbosity_name, 'String', new_verbosity);
    handles.NameOfVerbosity = new_verbosity;
    guidata(hObject,handles);
% Hints: contents = get(hObject,'String') returns verbosity contents as cell array
%        contents{get(hObject,'Value')} returns selected item from verbosity


% --- Executes during object creation, after setting all properties.
function verbosity_CreateFcn(hObject, eventdata, handles)
% hObject    handle to verbosity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.verbosity_contents = get(hObject,'String');
    guidata(hObject,handles);
% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function verbosity_name_Callback(hObject, eventdata, handles)
% hObject    handle to verbosity_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of verbosity_name as text
%        str2double(get(hObject,'String')) returns contents of verbosity_name as a double
    handles.NameOfVerbosity = get(hObject,'String');
    guidata(hObject,handles);
    

% --- Executes during object creation, after setting all properties.
function verbosity_name_CreateFcn(hObject, eventdata, handles)
% hObject    handle to verbosity_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.verbosity_name = hObject;
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_T_Callback(hObject, eventdata, handles)
% hObject    handle to delta_T (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_T as text
%        str2double(get(hObject,'String')) returns contents of delta_T as a double
    handles.delta_t_value = str2double(get(hObject,'String'));
    
    guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function delta_T_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_T (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.delta_T = hObject;
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function IC_multiple_Callback(hObject, eventdata, handles)
% hObject    handle to IC_multiple (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
         newval = get(hObject,'Value'); % get the new slider value
         handles.multiplier_value = newval;
        guidata(hObject,handles); %save the handles struct updates
        set(handles.IC_multiple_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_multiple_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_multiple (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.multiplier_min = get(hObject,'Min'); %make the slider min, 
     handles.multiplier_max = get(hObject,'Max');%max available to other functions
     handles.IC_multiple = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function IC_multiple_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_multiple_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.multiplier_max) || (newval < handles.multiplier_min),
          %retrieve slider value and store in handles
            handles.multiplier_value = get(handles.IC_multiple,'Value');
             set(hObject,'String',num2str(handles.multiplier_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
             guidata(hObject,handles); 
      else %use new value to override slider value
          set(handles.IC_multiple,'Value',newval)%set slider to new value
            handles.multiplier_value =  newval;%save new value as IC parameter 
            guidata(hObject,handles);
      end
% Hints: get(hObject,'String') returns contents of IC_multiple_value as text
%        str2double(get(hObject,'String')) returns contents of IC_multiple_value as a double


% --- Executes during object creation, after setting all properties.
function IC_multiple_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_multiple_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_multiple_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
    
    
%%%%%%%%%%%%%%%%%% THROTTLES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
% --- Executes on slider movement.
function IC_throttle_Callback(hObject, eventdata, handles)
% hObject    handle to IC_throttle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
          
         newval = get(hObject,'Value'); % get the new slider value
         handles.throttle_value = newval;
        guidata(hObject,handles); %save the handles struct updates
        set(handles.IC_throttle_value,'String',newval);%pass the new value to the textbox
 
% function IC_throttle_Set(handles)
%     handles.ValueOfThrottleSlider = handles.ValueOfThrottleValue;
%     set(handles.IC_throttle,'Value', handles.ValueOfThrottleSlider, 'Value', 1);
%     guidata(hObject,handles);
    
% --- Executes during object creation, after setting all properties.
function IC_throttle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_throttle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.throttle_min = get(hObject,'Min'); %make the slider min, 
     handles.throttle_max = get(hObject,'Max');%max available to other functions
     handles.IC_throttle = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function IC_throttle_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_throttle_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
     newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.throttle_max) || (newval < handles.throttle_min),
          %retrieve slider value and store in handles
            handles.throttle_value = get(handles.IC_throttle,'Value');
             set(hObject,'String',num2str(handles.throttle_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
             guidata(hObject,handles); 
      else %use new value to override slider value
          set(handles.IC_throttle,'Value',newval)%set slider to new value
            handles.throttle_value =  newval;%save new value as IC parameter 
            guidata(hObject,handles);
      end
% Hints: get(hObject,'String') returns contents of IC_throttle_value as text
%        str2double(get(hObject,'String')) returns contents of
%        IC_throttle_value as a double
        

% --- Executes during object creation, after setting all properties.
function IC_throttle_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_throttle_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_throttle_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%% AILERON
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function IC_aileron_Callback(hObject, eventdata, handles)
% hObject    handle to ic_aileron (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.aileron_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_aileron_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_aileron_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ic_aileron (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.aileron_min = get(hObject,'Min'); %make the slider min, 
    handles.aileron_max = get(hObject,'Max');%max available to other functions
    handles.IC_aileron = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function IC_aileron_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_aileron_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.aileron_max) || (newval < handles.aileron_min),
          %retrieve slider value and store in handles
            handles.aileron_value = get(handles.IC_aileron,'Value');
             set(hObject,'String',num2str(handles.aileron_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_aileron,'Value',newval);%set slider to new value
            handles.aileron_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_aileron_value as text
%        str2double(get(hObject,'String')) returns contents of IC_aileron_value as a double


% --- Executes during object creation, after setting all properties.
function IC_aileron_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_aileron_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_aileron_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%% ELEVATOR
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function IC_elevator_Callback(hObject, eventdata, handles)
% hObject    handle to IC_elevator (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.elevator_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_elevator_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_elevator_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_elevator (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.elevator_min = get(hObject,'Min'); %make the slider min, 
    handles.elevator_max = get(hObject,'Max');%max available to other functions
    handles.IC_elevator = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function IC_elevator_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_elevator_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.elevator_max) || (newval < handles.elevator_min),
          %retrieve slider value and store in handles
            handles.elevator_value = get(handles.IC_elevator,'Value');
             set(hObject,'String',num2str(handles.elevator_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_elevator,'Value',newval);%set slider to new value
            handles.elevator_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_elevator_value as text
%        str2double(get(hObject,'String')) returns contents of IC_elevator_value as a double


% --- Executes during object creation, after setting all properties.
function IC_elevator_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_elevator_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_elevator_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%% RUDDER
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function IC_rudder_Callback(hObject, eventdata, handles)
% hObject    handle to IC_rudder (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.rudder_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_rudder_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_rudder_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_rudder (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.rudder_min = get(hObject,'Min'); %make the slider min, 
    handles.rudder_max = get(hObject,'Max');%max available to other functions
    handles.IC_rudder = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function IC_rudder_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_rudder_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.rudder_max) || (newval < handles.rudder_min),
          %retrieve slider value and store in handles
            handles.rudder_value = get(handles.IC_rudder,'Value');
             set(hObject,'String',num2str(handles.rudder_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_rudder,'Value',newval);%set slider to new value
            handles.rudder_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_rudder_value as text
%        str2double(get(hObject,'String')) returns contents of IC_rudder_value as a double


% --- Executes during object creation, after setting all properties.
function IC_rudder_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_rudder_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_rudder_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%% MIXTURE
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function IC_mixture_Callback(hObject, eventdata, handles)
% hObject    handle to IC_mixture (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.mixture_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_mixture_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_mixture_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_mixture (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.mixture_min = get(hObject,'Min'); %make the slider min, 
    handles.mixture_max = get(hObject,'Max');%max available to other functions
    handles.IC_mixture = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function IC_mixture_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_mixture_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.mixture_max) || (newval < handles.mixture_min),
          %retrieve slider value and store in handles
            handles.mixture_value = get(handles.IC_mixture,'Value');
             set(hObject,'String',num2str(handles.mixture_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_mixture,'Value',newval);%set slider to new value
            handles.mixture_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_mixture_value as text
%        str2double(get(hObject,'String')) returns contents of IC_mixture_value as a double


% --- Executes during object creation, after setting all properties.
function IC_mixture_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_mixture_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_mixture_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%% GEAR
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function IC_gear_Callback(hObject, eventdata, handles)
% hObject    handle to IC_gear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.gear_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_gear_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_gear_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_gear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.gear_min = get(hObject,'Min'); %make the slider min, 
    handles.gear_max = get(hObject,'Max');%max available to other functions
    handles.IC_gear = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function IC_gear_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_gear_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.gear_max) || (newval < handles.gear_min),
          %retrieve slider value and store in handles
            handles.gear_value = get(handles.IC_gear,'Value');
             set(hObject,'String',num2str(handles.gear_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
             %guidata(hObject,handles); 
      else %use new value to override slider value
          set(handles.IC_gear,'Value',newval);%set slider to new value
            handles.gear_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end

      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_gear_value as text
%        str2double(get(hObject,'String')) returns contents of IC_gear_value as a double


% --- Executes during object creation, after setting all properties.
function IC_gear_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_gear_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_gear_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%% FLAPS
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function IC_flaps_Callback(hObject, eventdata, handles)
% hObject    handle to IC_flaps (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.flaps_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_flaps_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_flaps_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_flaps (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.flaps_min = get(hObject,'Min'); %make the slider min, 
    handles.flaps_max = get(hObject,'Max');%max available to other functions
    handles.IC_flaps = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function IC_flaps_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_flaps_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.flaps_max) || (newval < handles.flaps_min),
          %retrieve slider value and store in handles
            handles.flaps_value = get(handles.IC_flaps,'Value');
             set(hObject,'String',num2str(handles.flaps_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_flaps,'Value',newval);%set slider to new value
            handles.flaps_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_flaps_value as text
%        str2double(get(hObject,'String')) returns contents of IC_flaps_value as a double


% --- Executes during object creation, after setting all properties.
function IC_flaps_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_flaps_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_flaps_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%% SET RUNNING
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on button press in IC_set_run.
function IC_set_run_Callback(hObject, eventdata, handles)
% hObject    handle to IC_set_run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.set_run_value = get(hObject,'Value'); %returns toggle state of IC_set_run
    guidata(hObject,handles);
% Hint: get(hObject,'Value') returns toggle state of IC_set_run


% --- Executes during object creation, after setting all properties.
function IC_set_run_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_set_run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_set_run = hObject;
    guidata(hObject,handles);
%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%% U
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function IC_u_Callback(hObject, eventdata, handles)
% hObject    handle to IC_u (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.u_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_u_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_u_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_u (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.u_min = get(hObject,'Min'); %make the slider min, 
    handles.u_max = get(hObject,'Max');%max available to other functions
    handles.IC_u = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function IC_u_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_u_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.u_max) || (newval < handles.u_min)
          %retrieve slider value and store in handles
            handles.u_value = get(handles.IC_u,'Value');
             set(hObject,'String',num2str(handles.u_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
             %guidata(hObject,handles); 
      else %use new value to override slider value
          set(handles.IC_u,'Value',newval)%set slider to new value
            handles.u_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_u_value as text
%        str2double(get(hObject,'String')) returns contents of IC_u_value as a double


% --- Executes during object creation, after setting all properties.
function IC_u_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_u_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_u_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% U
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%% V
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function IC_v_Callback(hObject, eventdata, handles)
% hObject    handle to IC_v (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.v_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_v_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_v_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_v (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.v_min = get(hObject,'Min'); %make the slider min, 
    handles.v_max = get(hObject,'Max');%max available to other function
    handles.IC_v = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function IC_v_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_v_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.v_max) || (newval < handles.v_min),
          %retrieve slider value and store in handles
            handles.v_value = get(handles.IC_v,'Value');
             set(hObject,'String',num2str(handles.v_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_v,'Value',newval);%set slider to new value
            handles.v_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_v_value as text
%        str2double(get(hObject,'String')) returns contents of IC_v_value as a double


% --- Executes during object creation, after setting all properties.
function IC_v_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_v_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_v_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% V
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%% W
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function IC_w_Callback(hObject, eventdata, handles)
% hObject    handle to IC_w (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.w_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_w_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_w_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_w (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.w_min = get(hObject,'Min'); %make the slider min, 
    handles.w_max = get(hObject,'Max');%max available to other functions
    handles.IC_w = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function IC_w_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_w_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.w_max) || (newval < handles.w_min),
          %retrieve slider value and store in handles
            handles.w_value = get(handles.IC_w,'Value');
             set(hObject,'String',num2str(handles.w_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
             
      else %use new value to override slider value
          set(handles.IC_w,'Value',newval);%set slider to new value
            handles.w_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles); 
% Hints: get(hObject,'String') returns contents of IC_w_value as text
%        str2double(get(hObject,'String')) returns contents of IC_w_value as a double


% --- Executes during object creation, after setting all properties.
function IC_w_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_w_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_w_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% W
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%% P
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function IC_p_Callback(hObject, eventdata, handles)
% hObject    handle to IC_p (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.p_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_p_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_p_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_p (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.p_min = get(hObject,'Min'); %make the slider min, 
    handles.p_max = get(hObject,'Max');%max available to other functions
    handles.IC_p = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function IC_p_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_p_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.p_max) || (newval < handles.p_min),
          %retrieve slider value and store in handles
            handles.p_value = get(handles.IC_p,'Value');
             set(hObject,'String',num2str(handles.p_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
             
      else %use new value to override slider value
          set(handles.IC_p,'Value',newval);%set slider to new value
            handles.p_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
       guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_p_value as text
%        str2double(get(hObject,'String')) returns contents of IC_p_value as a double


% --- Executes during object creation, after setting all properties.
function IC_p_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_p_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_p_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% P
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%% Q
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% --- Executes on slider movement.
function IC_q_Callback(hObject, eventdata, handles)
% hObject    handle to IC_q (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.q_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_q_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_q_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_q (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.q_min = get(hObject,'Min'); %make the slider min, 
    handles.q_max = get(hObject,'Max');%max available to other functions
    handles.IC_q = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function IC_q_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_q_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.q_max) || (newval < handles.q_min),
          %retrieve slider value and store in handles
            handles.q_value = get(handles.IC_q,'Value');
             set(hObject,'String',num2str(handles.q_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_q,'Value',newval);%set slider to new value
            handles.q_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_q_value as text
%        str2double(get(hObject,'String')) returns contents of IC_q_value as a double


% --- Executes during object creation, after setting all properties.
function IC_q_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_q_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_q_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% Q
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%% R
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% --- Executes on slider movement.
function IC_r_Callback(hObject, eventdata, handles)
% hObject    handle to IC_r (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.r_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_r_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_r_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_r (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.r_min = get(hObject,'Min'); %make the slider min, 
    handles.r_max = get(hObject,'Max');%max available to other functions
    handles.IC_r = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function IC_r_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_r_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.r_max) || (newval < handles.r_min),
          %retrieve slider value and store in handles
            handles.r_value = get(handles.IC_r,'Value');
             set(hObject,'String',num2str(handles.r_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_r,'Value',newval);%set slider to new value
            handles.r_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_r_value as text
%        str2double(get(hObject,'String')) returns contents of IC_r_value as a double


% --- Executes during object creation, after setting all properties.
function IC_r_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_r_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_r_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% R
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%% H
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- Executes on slider movement.
function IC_h_Callback(hObject, eventdata, handles)
% hObject    handle to IC_h (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.h_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_h_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_h_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_h (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.h_min = get(hObject,'Min'); %make the slider min, 
    handles.h_max = get(hObject,'Max');%max available to other functions
     handles.IC_h = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function IC_h_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_h_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.h_max) || (newval < handles.h_min),
          %retrieve slider value and store in handles
            handles.h_value = get(handles.IC_h,'Value');
             set(hObject,'String',num2str(handles.h_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_h,'Value',newval);%set slider to new value
            handles.h_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_h_value as text
%        str2double(get(hObject,'String')) returns contents of IC_h_value as a double


% --- Executes during object creation, after setting all properties.
function IC_h_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_h_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_h_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% H
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%% PHI
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% --- Executes on slider movement.
function IC_phi_Callback(hObject, eventdata, handles)
% hObject    handle to IC_phi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.phi_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_phi_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_phi_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_phi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.phi_min = get(hObject,'Min'); %make the slider min, 
    handles.phi_max = get(hObject,'Max');%max available to other functions
     handles.IC_phi = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function IC_phi_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_phi_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.phi_max) || (newval < handles.phi_min),
          %retrieve slider value and store in handles
            handles.phi_value = get(handles.IC_phi,'Value');
             set(hObject,'String',num2str(handles.phi_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_phi,'Value',newval);%set slider to new value
            handles.phi_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_phi_value as text
%        str2double(get(hObject,'String')) returns contents of IC_phi_value as a double


% --- Executes during object creation, after setting all properties.
function IC_phi_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_phi_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_phi_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% PHI
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%%%%%%%%%%%%%%%%%% THETA
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% --- Executes on slider movement.
function IC_theta_Callback(hObject, eventdata, handles)
% hObject    handle to IC_theta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.theta_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_theta_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_theta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_theta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.theta_min = get(hObject,'Min'); %make the slider min, 
    handles.theta_max = get(hObject,'Max');%max available to other functions
     handles.IC_theta = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function IC_theta_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_theta_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.theta_max) || (newval < handles.theta_min),
          %retrieve slider value and store in handles
            handles.theta_value = get(handles.IC_theta,'Value');
             set(hObject,'String',num2str(handles.theta_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_theta,'Value',newval);%set slider to new value
            handles.theta_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_theta_value as text
%        str2double(get(hObject,'String')) returns contents of IC_theta_value as a double


% --- Executes during object creation, after setting all properties.
function IC_theta_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_theta_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_theta_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% THETA
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%% PSI
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- Executes on slider movement.
function IC_psi_Callback(hObject, eventdata, handles)
% hObject    handle to IC_psi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.psi_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_psi_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_psi_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_psi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.psi_min = get(hObject,'Min'); %make the slider min, 
    handles.psi_max = get(hObject,'Max');%max available to other functions
     handles.IC_psi = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function IC_psi_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_psi_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.psi_max) || (newval < handles.psi_min),
          %retrieve slider value and store in handles
            handles.psi_value = get(handles.IC_psi,'Value');
             set(hObject,'String',num2str(handles.psi_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_psi,'Value',newval);%set slider to new value
            handles.psi_value =  newval;%save new value as IC parameter 
           %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_psi_value as text
%        str2double(get(hObject,'String')) returns contents of IC_psi_value as a double


% --- Executes during object creation, after setting all properties.
function IC_psi_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_psi_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_psi_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% PSI
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%LAT
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function IC_lat_Callback(hObject, eventdata, handles)
% hObject    handle to IC_lat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.lat_value = str2double(get(hObject,'String')); %returns contents of IC_lat as a double 
    guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_lat as text
%        str2double(get(hObject,'String')) returns contents of IC_lat as a double


% --- Executes during object creation, after setting all properties.
function IC_lat_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_lat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_lat = hObject;
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% LAT
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%% LONG
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function IC_long_Callback(hObject, eventdata, handles)
% hObject    handle to IC_long (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.long_value = str2double(get(hObject,'String')) %returns contents of IC_long as a double  
    guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_long as text
%        str2double(get(hObject,'String')) returns contents of IC_long as a double


% --- Executes during object creation, after setting all properties.
function IC_long_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_long (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_long = hObject;
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% LONG
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% --- Executes on button press in init_model.
% --- Sets the model IC parameters
function init_model_Callback(hObject, eventdata, handles)
% hObject    handle to init_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 
    import_data('model_data')% load model_data.mat file for initial control settings
    
    %parameters must be set as strings with brackets.  Spaces are embedded
    %to aid in parsing the initialization parameters.
    Sp = ' ';
    Sb = '[';
    Se = ']';
    u = num2str(handles.u_value);
    v = num2str(handles.v_value);
    w = num2str(handles.w_value);
    p = num2str(handles.p_value);
    q = num2str(handles.q_value);
    r = num2str(handles.r_value);
    long = num2str(handles.long_value);
    lat = num2str(handles.lat_value);
    h = num2str(handles.h_value);
    phi = num2str(handles.phi_value);
    theta = num2str(handles.theta_value);
    psi = num2str(handles.psi_value);
    thr = num2str(handles.throttle_value); 
    ail = num2str(handles.aileron_value); 
    el = num2str(handles.elevator_value);
    data.elevator = handles.elevator_value;
    rud = num2str(handles.rudder_value); 
    mix = num2str(handles.mixture_value); 
    run_set = num2str(handles.set_run_value); 
    flap = num2str(handles.flaps_value); 
    gear = num2str(handles.gear_value);
   
    % set the control's step output inital values to match the inital
    % controls parameters.
    assignin('base', 'init_throttle', handles.throttle_value)
    assignin('base', 'init_aileron', handles.aileron_value)
    assignin('base', 'init_elevator', handles.elevator_value)
    assignin('base', 'init_rudder', handles.rudder_value)
    assignin('base', 'init_mixture', handles.mixture_value)
    assignin('base', 'init_run_set', handles.set_run_value)
    assignin('base', 'init_flaps', handles.flaps_value)
    assignin('base', 'init_gear', handles.gear_value)
    set_param(handles.ModelPath,'ac_name',...
                            handles.NameOfAircraft);
    set_param(handles.ModelPath,'IC_states',...
                            [Sb Sp u Sp v Sp w Sp p Sp q Sp r Sp h Sp long Sp lat Sp phi Sp theta Sp psi Sp Se]);%                   
    set_param(handles.ModelPath,'IC_controls',...
                            [Sb Sp thr Sp ail Sp el Sp rud Sp mix Sp run_set Sp flap Sp gear Sp Se]);                   
    set_param(handles.ModelPath,'IC_delta_t',...
                             num2str(handles.delta_t_value));
    set_param(handles.ModelPath,'IC_verbosity',...
                            handles.NameOfVerbosity);                    
    set_param(handles.ModelPath,'IC_sim_multiplier', num2str(handles.multiplier_value));
%     set_param(handles.ModelPath,'IC_int_override', num2str(handles.set_int_override));
% --- Executes during object creation, after setting all properties.
function init_model_CreateFcn(hObject, eventdata, handles)
% hObject    handle to init_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in clear_SF.
function clear_SF_Callback(hObject, eventdata, handles)
% hObject    handle to clear_SF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    clearSF;
    

% --- Executes during object creation, after setting all properties.
function text1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in help_button.
function help_button_Callback(hObject, eventdata, handles)
% hObject    handle to help_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    open('JSBSimGUI_Help')

% 
% % --- Executes on button press in IC_int_override_value.
% function IC_int_override_value_Callback(hObject, eventdata, handles)
% % hObject    handle to IC_int_override_value (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
%     handles.set_int_override = get(hObject,'Value'); %returns toggle state of IC_set_run
%     guidata(hObject,handles);
% % Hint: get(hObject,'Value') returns toggle state of IC_int_override_value
% 
% 
% % --- Executes during object creation, after setting all properties.
% function IC_int_override_value_CreateFcn(hObject, eventdata, handles)
% % hObject    handle to IC_int_override_value (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    empty - handles not created until after all CreateFcns called
%     handles.IC_int_override_value = hObject;
%     guidata(hObject,handles);



