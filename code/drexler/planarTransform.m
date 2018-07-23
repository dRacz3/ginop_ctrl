function T=planarTransform(angle,position,scale)

T = scale*[planarRot(angle) position; 0 0 1];

end