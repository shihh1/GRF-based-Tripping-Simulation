%function trippasttime = trip(cws,leftright,accel,remoteobj,bodyweight, clock)
function str = trip_v2(cws,leftright,accel,remoteobj,bodyweight)
% Load the SDK
% establish real time communication with Vicon Nexus

% the abs(Fz) needs to be between 1/5 and 1/4 times body weight  
% abs(Fz) has to be greater than the value of the frame that is 0.01 sec
% before

upperthresh = bodyweight * 9.8 * 0.25; 
lowerthresh = bodyweight * 9.8 * 0.2; 

setenv('MW_MINGW64_LOC','C:\mingw-w64\x86_64-5.3.0-posix-seh-rt_v4-rev0\mingw64');

%fprintf( 'Loading SDK...' );
Client.LoadViconDataStreamSDK();
%fprintf( 'done\n' );

% Program options
HostName = 'localhost:801';

% Make a new client
MyClient = Client();

% Connect to a server
%fprintf( 'Connecting to %s ...', HostName );
while ~MyClient.IsConnected().Connected
  % Direct connection
  MyClient.Connect( HostName );
  %fprintf( '.' );
end
%fprintf( '\n' );


MyClient.EnableDeviceData();

% Set the streaming mode
MyClient.SetStreamMode( StreamMode.ClientPull );
% MyClient.SetStreamMode( StreamMode.ClientPullPreFetch );
% MyClient.SetStreamMode( StreamMode.ServerPush );

counter = 1;
zforce_vector = zeros(1,20);

%lowpoint = cws + 0.05 * accel * -1;
%highpoint = cws + 0.22 * accel;
lowpoint = cws + 0.05 * accel * cws * -1;
highpoint = cws + 0.22 * accel * cws;
orispeed = [cws cws cws cws];
if (leftright == 'L')
    ForcePlateIndex = 1;
    lowspeed = [cws lowpoint cws lowpoint];
    highspeed = [cws highpoint cws highpoint];
else
    ForcePlateIndex = 2;
    lowspeed = [lowpoint cws lowpoint cws];
    highspeed = [highpoint cws highpoint cws];
end


while 1
    % Get a frame
    %fprintf( 'Waiting for new frame...' );
    while MyClient.GetFrame().Result.Value ~= Result.Success
        %fprintf( '.' );
    end% while
    %fprintf( '\n' );  
    Output_GetGlobalForceVector = MyClient.GetGlobalForceVector( ForcePlateIndex );
    zforce = abs(Output_GetGlobalForceVector.ForceVector(3)); 
    if (counter <= 20)
        zforce_vector(counter) = zforce;
        counter = counter + 1;
    else % counter > 20
        zforce_previous = zforce_vector(1);
        zforce_vector = zforce_vector(2:20);
        zforce_vector(20) = zforce;
        counter = counter + 1;
        if ((zforce > lowerthresh) && (zforce < upperthresh) && (zforce > zforce_previous)) % trigger the event
            % add this to plot the graph
            %trippasttime = toc(clock); 
            tm_set_new(remoteobj,lowspeed,accel);
            pause(0.05);
            tm_set_new(remoteobj,highspeed,accel);
            pause(0.27);
            tm_set_new(remoteobj,orispeed,accel);
            pause(0.22);
            %fprintf("%s %u %f \n", leftright, accel, zforce);
            Output_GetFrameNumber = MyClient.GetFrameNumber();
            %fprintf( 'Frame Number: %d\n', Output_GetFrameNumber.FrameNumber );
            %fprintf("Trip on %s Belt with acceleration %u at force %f \n", leftright, accel, zforce);
            str = "Direction= " + leftright + " Accel= " + accel + " ZForce= " + zforce + " FrameNumber= " + Output_GetFrameNumber.FrameNumber;
            break; % break the while loop
        end % end if trigger conditions
    end % end of counter
end % end of while 1

% Disconnect and dispose
MyClient.Disconnect();

% Unload the SDK
%fprintf( 'Unloading SDK...' );
Client.UnloadViconDataStreamSDK();
%fprintf( 'done\n' );

end