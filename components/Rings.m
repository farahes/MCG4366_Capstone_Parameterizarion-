% =========================================================
% RETAINING RINGS
% =========================================================
% Parameterized size for the shaft external retaining rings
% Sizes are from the McMaster Carr catalogue for external retaining rings

classdef Rings

methods (Static)

    % input small shaft diameter [mm] and output the dimensions for the
    % retatining ring to text file for solidworks integration
    function getRRing(d_s)

        % dimensions in [mm]
        if d_s == 18
            groove_d = 17.00;
            shaft_od = 18.00;
            inner_d = 16.50;
            top_width = 2.40;
            lug_height = 3.90;
            hole_d = 2.00;
            lug_inner_angle = 2;    % [deg]
            lug_width = 5.00;
            lug_height2 = 3.184;
            lug_radius = 0.5163;
            edge_width = 1.6667;
            edge_width2 = 1.6667;
            edge_radius = 2.0651;
            angle = 120;    % [deg]
            groove_t = 1.3;
            ring_thickness = 1.2;
        elseif d_s == 20
            groove_d = 19.00;
            shaft_od = 20.00;
            inner_d = 18.50;
            top_width = 2.60;
            lug_height = 4.00;
            hole_d = 2.00;
            lug_inner_angle = 2;    % [deg]
            lug_width = 5.00;
            lug_height2 = 3.1774;
            lug_radius = 0.5035;
            edge_width = 1.6667;
            edge_width2 = 1.6667;
            edge_radius = 2.0139;
            angle = 120;    % [deg]
            groove_t = 1.3;
            ring_thickness = 1.2;
        elseif d_s == 22
            groove_d = 21.00;
            shaft_od = 22.00;
            inner_d = 20.50;
            top_width = 2.80;
            lug_height = 4.20;
            hole_d = 2.00;
            lug_inner_angle = 2;    % [deg]
            lug_width = 5.00;
            lug_height2 = 3.2609;
            lug_radius = 0.4863;
            edge_width = 1.6667;
            edge_width2 = 1.6667;
            edge_radius = 1.9451;
            angle = 120;    % [deg]
            groove_t = 1.3;
            ring_thickness = 1.2;
        else
            groove_d = 23.90;
            shaft_od = 25.00;
            inner_d = 23.2;
            top_width = 3.00;
            lug_height = 4.40;
            hole_d = 2.00;
            lug_inner_angle = 2;    % [deg]
            lug_width = 5.00;
            lug_height2 = 3.3462;
            lug_radius = 0.4707;
            edge_width = 1.6667;
            edge_width2 = 1.6667;
            edge_radius = 1.8826;
            angle = 120;    % [deg]
            groove_t = 1.3;
            ring_thickness = 1.2;
        end

        % create and export dimensions to text file
        basePath = fileparts(mfilename('fullpath'));
        filePath = fullfile(basePath, '..', '..', 'Solidworks', 'Equations', 'retainingring.txt');
        file = fopen(filePath, 'w');
        if file == -1
            error('Could not create retainingring.txt');
        end

        fprintf(file, '"groove_d"=%.2f\n', groove_d);
        fprintf(file, '"shaft_od"=%.2f\n', shaft_od);
        fprintf(file, '"inner_d"=%.2f\n', inner_d);
        fprintf(file, '"top_width"=%.2f\n', top_width);
        fprintf(file, '"lug_height"=%.2f\n', lug_height);
        fprintf(file, '"hole_d"=%.2f\n', hole_d);
        fprintf(file, '"lug_inner_angle"=%.2f\n', lug_inner_angle);
        fprintf(file, '"lug_width"=%.2f\n', lug_width);
        fprintf(file, '"lug_height2"=%.2f\n', lug_height2);
        fprintf(file, '"lug_radius"=%.2f\n', lug_radius);
        fprintf(file, '"edge_width"=%.2f\n', edge_width);
        fprintf(file, '"edge_width2"=%.2f\n', edge_width2);
        fprintf(file, '"edge_radius"=%.2f\n', edge_radius);
        fprintf(file, '"angle"=%.2f\n', angle);
        fprintf(file, '"groove_t"=%.2f\n', groove_t);
        fprintf(file, '"ring_thickness"=%.2f\n', ring_thickness);
        fclose(file);

    end
end

end