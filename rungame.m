close all

team1 = struct('name','YoniBot','color',rand(1,3),'strategy',@robostrategy_yoni);
team2 = struct('name','Good','color',rand(1,3),'strategy',@robostrategy_good);
robotgame_main(team1,team2);

close all
team1 = struct('name','Team A','color',rand(1,3),'strategy',@robostrategy_good);
team2 = struct('name','Team B','color',rand(1,3),'strategy',@robostrategy_rand);
robotgame_main(team1,team2);

close all
team1 = struct('name','Team A','color',rand(1,3),'strategy',@robostrategy_good);
team2 = struct('name','Team B','color',rand(1,3),'strategy',@robostrategy_direct);
robotgame_main(team1,team2);

