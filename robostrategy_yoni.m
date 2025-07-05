function [move, mem] = robostrategy_yoni(env, mem)
if nargin < 2 || isempty(mem)
    mem = struct();
end
if ~isfield(mem, 'prevPos')
    mem.prevPos = env.info.myPos;
end
if ~isfield(mem, 'prevFoePos')
    mem.prevFoePos = env.info.opPos;
end

  botPos = env.info.myPos; % [x y]
    foePos = env.info.opPos; % [x y]
    botFuel = env.info.fuel;
    foeFuel = env.info.fuel_op;
    fuelMargin = botFuel - foeFuel;
    distToFoe = norm(botPos - foePos);
    minesPos = env.mines.mPos; % [x y]
    fuelsPos = env.fuels.fPos; % [x y]
    minesExist = env.mines.mExist; % logical 
    fuelsExist = env.fuels.fExist; % logical
    radiusMF = env.basic.rMF; % 0.3
    radiusBot = env.basic.rRbt; % 0.5
    maxStep = env.basic.lmax;
    fuelValue = env.fuels.fScr(1);

    fDist = inf;
    fIdx = -1;
    foeCloser = false;
    fuelsLeft = sum(fuelsExist);
    threatDetected = fuelMargin <= 0;
    dxToFoe = foePos(1) - botPos(1);
    dyToFoe = foePos(2) - botPos(2);
    angleToFoe = atan2(dyToFoe, dxToFoe);

    % Predict foe future position
    foeDir = foePos - mem.prevFoePos;
    predictedFoePos = foePos + foeDir;

    % Attack mode: block if feasible
    for i = 1:env.fuels.nFuel
        if fuelsExist(i)
            myDist = norm(fuelsPos(i,:) - botPos);
            predictedFoeDist = norm(fuelsPos(i,:) - predictedFoePos);
            if predictedFoeDist < 3 && myDist < predictedFoeDist - 0.4
                angleToFuel = atan2(fuelsPos(i,2)-botPos(2), fuelsPos(i,1)-botPos(1));
                move = avoidMines(angleToFuel);
                move = wallCheck(move); move = filterDeadlock(move); mem.prevPos = botPos; mem.prevFoePos = foePos; return;
            end
        end
    end

    for i = 1:env.fuels.nFuel
        if fuelsExist(i)
            dx = fuelsPos(i, 1) - botPos(1);
            dy = fuelsPos(i, 2) - botPos(2);
            tempDist = sqrt(dx^2 + dy^2);
            if tempDist < fDist
                dxFoe = fuelsPos(i, 1) - predictedFoePos(1);
                dyFoe = fuelsPos(i, 2) - predictedFoePos(2);
                foeDist = sqrt(dxFoe^2 + dyFoe^2);
                if foeDist < tempDist && ~foeCloser
                    foeCloser = true;
                    continue;
                end
                fDist = tempDist;
                fIdx = i;
            end
        end
    end

    x = fuelsPos(fuelsExist,1);
    y = fuelsPos(fuelsExist,2);
    [denseDiff, quadMax] = getDensestQuadrant(x,y);
    myQuad = getQuadrant(botPos(1),botPos(2));
    foeQuad = getQuadrant(foePos(1),foePos(2));
    pursueDensity = (foeQuad ~= quadMax) && denseDiff > getQuadrantDensity(myQuad) + 2;

    if fuelMargin > 0 && distToFoe < fDist
        if fuelMargin >= 30 && distToFoe <= maxStep
            move = maxStep * [cos(angleToFoe), sin(angleToFoe)];
            move = wallCheck(move); move = filterDeadlock(move); mem.prevPos = botPos; mem.prevFoePos = foePos; return;
        end
        move = avoidMines(angleToFoe);
        move = wallCheck(move); move = filterDeadlock(move); mem.prevPos = botPos; mem.prevFoePos = foePos; return;
    end

    if fIdx == -1 && botFuel <= foeFuel
       move = [0 0];
       return;
    end

    if threatDetected && distToFoe <= 2*(maxStep + radiusBot)
        dx = foePos(1) - botPos(1);
        dy = foePos(2) - botPos(2);
        escapeAngle = atan2(dy, dx) + pi;
        optimalAngle = -1;
        if distToFoe > maxStep + radiusBot
            angleRange = pi;
        else
            angleRange = pi/2;
        end
        for j = 1:env.fuels.nFuel
            dist = inf;
            if fuelsExist(j)
                tdx = fuelsPos(j, 1) - botPos(1);
                tdy = fuelsPos(j, 2) - botPos(2);
                tDist = sqrt(tdx^2 + tdy^2);
                if tDist < dist
                    tAngle = atan2(tdy, tdx);
                    dist = tDist;
                end
                if abs(tAngle) < angleRange
                    optimalAngle = tAngle;
                else
                    optimalAngle = escapeAngle;
                end
            end
        end
        move = avoidMines(optimalAngle);
        move = wallCheck(move); move = filterDeadlock(move); mem.prevPos = botPos; mem.prevFoePos = foePos; return;
    end

    if pursueDensity && fDist > maxStep * 5 && myQuad ~= quadMax
        if quadMax == 1 || quadMax == 3
            dx = 2.5 - botPos(1);
        else
            dx = 7.5 - botPos(1);
        end
        if quadMax == 1 || quadMax == 2
            dy = 2.5 - botPos(2);
        else
            dy = 7.5 - botPos(2);
        end
        dir = atan2(dy,dx);
        move = avoidMines(dir);
        move = wallCheck(move); move = filterDeadlock(move); mem.prevPos = botPos; mem.prevFoePos = foePos; return;
    end

    if fIdx ~= -1
        dx = fuelsPos(fIdx, 1) - botPos(1);
        dy = fuelsPos(fIdx, 2) - botPos(2);
        dir = atan2(dy, dx);
        move = avoidMines(dir);
        move = wallCheck(move); move = filterDeadlock(move); mem.prevPos = botPos; mem.prevFoePos = foePos; return;
    end

    function move = avoidMines(angle)
        if any(minesExist)
            mDist = inf;
            mIdx = -1;
            for i = 1:env.mines.nMine
                if minesExist(i)
                    dx = minesPos(i, 1) - botPos(1);
                    dy = minesPos(i, 2) - botPos(2);
                    tempDist = sqrt(dx^2 + dy^2);
                    if tempDist < mDist
                        mDist = tempDist;
                        mIdx = i;
                    end
                end
            end
        end
        if mIdx ~= -1 && mDist <= maxStep + radiusMF
            dx = minesPos(mIdx, 1) - botPos(1);
            dy = minesPos(mIdx, 2) - botPos(2);
            mAngle = atan2(dy, dx);
            angleCW = mAngle - pi/2;
            angleCCW = mAngle + pi/2;
            if angleDiff(angle, angleCW) < angleDiff(angle, angleCCW)
                move = [maxStep * cos(angleCW), maxStep * sin(angleCW)];
                move = wallCheck(move); move = filterDeadlock(move); mem.prevPos = botPos; return;
            else
                move = [maxStep * cos(angleCCW), maxStep * sin(angleCCW)];
                move = wallCheck(move); move = filterDeadlock(move); mem.prevPos = botPos; return;
            end
        else
            move = [maxStep * cos(angle), maxStep * sin(angle)];
            move = wallCheck(move); move = filterDeadlock(move); mem.prevPos = botPos; return;
        end
    end

    function move = wallCheck(move)
        newPos = botPos + move;
        if newPos(1) >= 10
            move(1) = 10;
        end
        if newPos(1) <= 0
            move(1) = 0;
        end
        if newPos(2) >= 10
            move(2) = 10;
        end
        if newPos(2) <= 0
            move(2) = 0;
        end
    end

    function diff = angleDiff(a1, a2)
        diff = abs(mod(a1 - a2 + pi, 2 * pi) - pi);
    end

    function quadrant = getQuadrant(x, y)
        if x < 5 && y < 5
            quadrant = 1;
            return;
        end
        if x > 5 && y < 5
            quadrant = 2;
            return;
        end
        if x < 5 && y > 5
            quadrant = 3;
            return;
        end
        if x > 5 && y > 5
            quadrant = 4;
            return;
        end
    end

    function [diffCount, densest] = getDensestQuadrant(x, y)
        q1 = sum((x > 0) & (x <= 5) & (y > 0) & (y <= 5));
        q2 = sum((x > 5) & (x <= 10) & (y > 0) & (y <= 5));
        q3 = sum((x > 0) & (x <= 5) & (y > 5) & (y <= 10));
        q4 = sum((x > 5) & (x <= 10) & (y > 5) & (y <= 10));
        [diffCount, idx] = max([q1, q2, q3, q4]);
        diffCount = diffCount - min([q1, q2, q3, q4]);
        densest = idx;
    end
function move = filterDeadlock(move)
    if norm(botPos + move - mem.prevPos) < 0.1
        angle = rand * 2 * pi;
        move = maxStep * [cos(angle), sin(angle)];
    end
end


    function density = getQuadrantDensity(q)
        x = fuelsPos(fuelsExist, 1);
        y = fuelsPos(fuelsExist, 2);
        if q == 1
            density = sum((x > 0) & (x <= 5) & (y > 0) & (y <= 5));
            return;
        end
        if q == 2
            density = sum((x > 5) & (x <= 10) & (y > 0) & (y <= 5));
            return;
        end
        if q == 3
            density = sum((x > 0) & (x <= 5) & (y > 5) & (y <= 10));
            return;
        end
        if q == 4
            density = sum((x > 5) & (x <= 10) & (y > 5) & (y <= 10));
            return;
        end
    end
end
