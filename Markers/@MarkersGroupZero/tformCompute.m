function [SO3, SE3] = tformCompute(obj)
%tformCompute Compute the homogeneous transformation matrix of each frame
%based on the posture of the first frame. Note that only THREE Markers are
%considerred which means that M >= 3.
%   SO3: 3 x 3 x N, the rotation transformation matrices
%   SE3: 4 x 4 x N, the homogeneous transformation matrices
%   @ MarkersGroupZero

SO3 = repmat(eye(3),[1,1,obj.N]);
SE3 = repmat(eye(4),[1,1,obj.N]);
if obj.M >= 3
    tempMarkerPos = zeros(3,3);
    for i = 1:obj.N
        for j = 1:3
            tempMarkerPos(j,:) = obj.Markers(j).XYZ(i,:);
        end
        [tempSO3, tempP] = obj.threeMarkerPos(tempMarkerPos);
        SO3(:,:,i) = tempSO3;
        SE3(:,:,i) = SO3P2SE3(tempSO3,tempP);
    end
end

end

