function p = transform_to_global2d(p, R, t)
%% 2d
p(1:2,:) = R*p(1:2,:);

            % translate
p(1,:) = p(1,:) + t(1);
p(2,:) = p(2,:) + t(2);

end