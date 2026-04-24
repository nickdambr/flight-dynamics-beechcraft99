% export_figures.m
% Run the main analysis scripts and save each generated figure as a 200 DPI
% PNG in ../figures/, using clean semantic filenames.
%
% Usage (from src/):
%   addpath(genpath('../libs/airlib'))
%   export_figures
%
% Figure-number to filename mapping is defined below. If a figure number is
% missing (e.g. not produced by the current branch of the switch/case),
% that row is silently skipped.

close all

outdir = fullfile('..', 'figures');
if ~exist(outdir, 'dir'), mkdir(outdir); end

% ---------- Modal analysis (flight_dynamics_analysis.m) ----------
flight_dynamics_analysis

modal_figs = { ...
    1, 'pzmap_longitudinal'              ; ...   % Ch. 4.1
    2, 'argand_phugoid'                  ; ...   % Ch. 4.1.1
    3, 'argand_short_period'             ; ...   % Ch. 4.1.2
    4, 'pzmap_lateral'                   ; ...   % Ch. 4.2
    5, 'argand_spiral'                   ; ...   % Ch. 4.2.1
    6, 'argand_roll'                     ; ...   % Ch. 4.2.2
    7, 'argand_dutch_roll'               ; ...   % Ch. 4.2.3
    8, 'longitudinal_vs_lateral_poles'   ; ...   % comparison
    9, 'lateral_directional_stability'     ...   % Ch. 5.2.5
};

save_figs(modal_figs, outdir);

% ---------- Frequency response (bode_plots.m) ----------
% bode_plots also uses figure(9..13); clear first to avoid collisions.
close all
bode_plots

bode_figs = { ...
     9, 'bode_velocity'               ; ...   % Ch. 6.1
    10, 'bode_alpha_full'             ; ...   % Ch. 6.2 (complete TF)
    11, 'bode_pitch'                  ; ...   % Ch. 6.3
    12, 'bode_velocity_density_comparison' ; ...   % Ch. 6.1.1
    13, 'bode_alpha_approximated'       ...   % Ch. 6.2 (SP approximation)
};

save_figs(bode_figs, outdir);

fprintf('\nDone. Exported figures in %s\n', outdir);


function save_figs(map, outdir)
    for k = 1:size(map, 1)
        n  = map{k, 1};
        nm = map{k, 2};
        h  = findobj('Type', 'figure', 'Number', n);
        if isempty(h), continue, end
        exportgraphics(h, fullfile(outdir, [nm '.png']), 'Resolution', 200);
        fprintf('  exported %s.png\n', nm);
    end
end
