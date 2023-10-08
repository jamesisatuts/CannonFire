classdef Enviroment
    %ENVIROMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    methods (Static)
        
        function runner()
            close all
            clear all
            set(0,'DefaultFigureWindowStyle','docked')
            hold on  

            axis([-5,5, -5,5, 0,5] )
            
            
            %% place floor
          
            surf([-5,-5;5,5] ...
            ,[-5,5;-5,5] ...
            ,[0.0,0.0;0.0,0.0] ...
            ,'CData',imread('pirateShip.jpg') ...
            ,'FaceColor','texturemap');

            %% place cannon 
            CannonPlacement = [-3.5  0 0.3];
           
            CannonBarrel = PlaceObject('CannonBarrel.ply',CannonPlacement);
            CannonStand = PlaceObject('CannonStand.ply', );
        end
        

        

        
    end
end

