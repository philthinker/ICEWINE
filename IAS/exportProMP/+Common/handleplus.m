classdef handleplus < handle
% 
% This class is an extension to the handle class, which allows for deep copies of objects. If OBJ is a member of
% "handleplus", a deep copy may be created using:
%
% >> OBJ_new = OBJ.copy;
% >> OBJ_new = OBJ.clone;  % Alias-method, same as OBJ.copy;
%
% It additionally implements a property "version", which I generally find very useful. It can be used while loading
% objects from disk to detect, if these are not up to date anymore. Since this property hidden and defineds as private,
% it may be redefined in any child-class and should not cause any problems.
%
% NOTE
% ====
% - This implementation (hopefully) also handles inherited properties, which are defined as 
%   "private" or "protected" in its superclass.
% - If properties contain child objects of the HANDLEPLUS-class, they are also deep copied.
%   If this is an unwanted behaviour, this needs to be changed manually.
% - This implementation DOES NOT handle cyclic references as discussed in this post:
%   <a href="matlab:web('mathforum.org/kb/message.jspa?messageID=7629086&tstart=0')"></a>.
%

% History
% =======
% 25.01.2012    Version 1 published to FEX
%
% Author
% ======
% Sebastian Hlz
% shoelz(at)geomar(dot)de
%
    properties (SetAccess=private,GetAccess=private,Hidden)
        version=1;
    end
    
    % === Additional methods
    methods
        function obj_out = clone(obj)
            % obj_new = OBJ.clone;
            %
            % Use this syntax to make a deep copy of an object OBJ, i.e. OBJ_OUT has the same field values,
            % but will not behave as a handle-copy of OBJ anymore.
            %
            meta = metaclass(obj);
            obj_out = feval(class(obj));
            s = warning('off', 'My_hgsetget:dimension');
            for i = 1:length(meta.Properties)
                prop = meta.Properties{i};
                if ~(prop.Dependent || prop.Constant) && ~(isempty(obj.(prop.Name)) && isempty(obj_out.(prop.Name)))
                    if isobject(obj.(prop.Name)) && isa(obj.(prop.Name),'handleplus')
                        obj_out.(prop.Name) = obj.(prop.Name).clone;
                    else
                        try
                            obj_out.(prop.Name) = obj.(prop.Name);
                        catch
                            warning('handleplus:copy', 'Problem copying property "%s"',prop.Name)
                        end
                    end
                end
                warning(s)
            end
            
            % Check lower levels ...
            for i = 1:length(meta.Properties)
                name = meta.Properties{i}.Name;
                obj_out.(name) = obj.(name);
            end
            
%             CheckSuperclasses(meta)
%             
%             % This function is called recursively ...
%             function CheckSuperclasses(List)
%                 for ii=1:length(List.Superclass{:})
%                     if ~isempty(List.Superclass{ii}.Superclass)
%                         CheckSuperclasses(List.Superclass{ii})
%                     end
%                     for jj=1:length(List.Superclass{ii}.Properties)
%                         prop_super = List.Superclass{ii}.Properties{jj}.Name;
%                         if ~strcmp(prop_super, props_child)
%                             obj_out.(prop_super) = obj.(prop_super);
%                         end
%                     end
%                 end
%             end
            
            
        end
        function obj_out = copy(obj)
            %
            % obj_new = obj.copy; % Same as clone-method.
            %
            % Use this syntax to make a deep copy of an object OBJ, i.e. OBJ_OUT has the same field values,
            % but will not behave as a handle-copy of OBJ anymore.
            %
            obj_out = obj.clone;
        end
    end
    
end