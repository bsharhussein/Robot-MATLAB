function [move, mem] = MyRobotStrategy2(env, mem)

%% === INIT ===
lmax      = env.basic.lmax;
start     = env.info.myPos;
myFuel    = env.info.fuel;
opFuel    = env.info.fuel_op;
opPos     = env.info.opPos;
mpos      = env.mines.mPos;
mineExist = env.mines.mExist;
mineRad   = env.basic.rMF;
fpos      = env.fuels.fPos;
fExist    = env.fuels.fExist;
numMines  = env.mines.nMine;
numFuels  = env.fuels.nFuel;
xyLim     = [0,10;0,10];

%% === MEMORY INIT ===
if isempty(mem)
    mem = struct(...
        'chase',         false, ...
        'chaseCount',    0,     ...
        'lastFuel',      myFuel,...
        'interceptAngle',0,     ...
        'stuckCount',    0,     ...
        'lastPos',       start  ...
    );
end

%% === TUNABLE PARAMETERS ===
ATTACK_FUEL_ADVANTAGE   = 70;   % attack if we have this much more fuel
FUEL_CRITICAL           = 50;   % emergency refuel threshold
FUEL_COMFORTABLE        = 80;   % safe refuel threshold
CHASE_MIN_TURNS         = 6;    % minimum turns to keep chasing
INTERCEPT_LOOKAHEAD     = 3;    % steps ahead to predict opponent
MINE_SAFE_MARGIN        = 0.4;  % extra clearance beyond mine radius
STUCK_THRESHOLD         = 0.05; % if we moved less than this, we're stuck

%% === DETECT IF STUCK ===
movedDist = getDist(start, mem.lastPos);
if movedDist < STUCK_THRESHOLD
    mem.stuckCount = mem.stuckCount + 1;
else
    mem.stuckCount = 0;
end
mem.lastPos = start;

%% === STEP 1: Score all fuel tanks (distance + value) ===
bestScore   = inf;
bestFuelIdx = -1;
for i = 1:numFuels
    if fExist(i) == 1
        d = getDist(start, fpos(i,:));
        % Prefer closer tanks; also prefer tanks far from opponent
        opToFuel = getDist(opPos, fpos(i,:));
        score = d - 0.3 * opToFuel;  % penalize tanks opponent is also chasing
        if score < bestScore
            bestScore   = score;
            bestFuelIdx = i;
        end
    end
end

%% === STEP 2: Predict opponent's next position (intercept logic) ===
% Simple linear prediction: assume opponent moves toward nearest fuel or us
opPredicted = opPos; % default: current position
if bestFuelIdx > 0
    opToFuel = getDist(opPos, fpos(bestFuelIdx,:));
    myToFuel = getDist(start,  fpos(bestFuelIdx,:));
    if opToFuel < myToFuel
        % Opponent likely heading to that fuel tank — predict ahead
        dirToFuel   = (fpos(bestFuelIdx,:) - opPos) / (opToFuel + 1e-6);
        opPredicted = opPos + dirToFuel * lmax * INTERCEPT_LOOKAHEAD;
    end
end

%% === STEP 3: Decide destination (state machine) ===
fuelAdvantage = myFuel - opFuel;
opDist        = getDist(start, opPos);

% Recalculate fuel rate lost per step using memory
fuelBurnRate = mem.lastFuel - myFuel;
mem.lastFuel = myFuel;

% Estimate steps to reach opponent
stepsToOp = opDist / lmax;

% Will we still have advantage when we reach opponent?
projectedAdvantage = fuelAdvantage - fuelBurnRate * stepsToOp;

% === ATTACK CONDITIONS ===
shouldAttack = ...
    (fuelAdvantage >= ATTACK_FUEL_ADVANTAGE && projectedAdvantage > 20) || ...
    (mem.chase && myFuel > opFuel + 15 && mem.chaseCount < CHASE_MIN_TURNS) || ...
    (fuelAdvantage > 25 && opDist < 2.0);

% === STUCK OVERRIDE: if stuck, force random escape direction ===
if mem.stuckCount >= 3
    % Pick a random direction away from mines
    angles   = linspace(0, 2*pi, 8);
    bestEsc  = start + [cos(angles(1)), sin(angles(1))] * lmax * 2;
    bestSafe = -inf;
    for k = 1:length(angles)
        candidate = start + [cos(angles(k)), sin(angles(k))] * lmax * 2;
        candidate(1) = max(0.2, min(9.8, candidate(1)));
        candidate(2) = max(0.2, min(9.8, candidate(2)));
        safeScore = minDistToMines(candidate, mpos, mineExist, numMines);
        if safeScore > bestSafe
            bestSafe = safeScore;
            bestEsc  = candidate;
        end
    end
    destination    = bestEsc;
    mem.stuckCount = 0;
    
elseif shouldAttack
    % === ATTACK: aim at predicted intercept position ===
    destination    = opPredicted;
    mem.chase      = true;
    mem.chaseCount = mem.chaseCount + 1;
    
else
    mem.chase      = false;
    mem.chaseCount = 0;
    
    % === THE ENDGAME PATCH (Empty Board Logic) ===
    if bestFuelIdx < 0 
        if myFuel > opFuel
            % We are winning! Charge the opponent to trigger the 5-unit end condition!
            destination = opPredicted;
        else
            % We are losing and there is no fuel. CONSERVE FUEL.
            if opDist < 6.5 
                % Opponent is getting close, run to a corner!
                destination = findSafestCorner(start, opPos, xyLim);
            else
                % Opponent is far away. Stand perfectly still to burn minimum fuel (2.0)
                destination = start; 
            end
        end
        
    % === STANDARD GAMEPLAY (Fuel exists) ===
    elseif myFuel < FUEL_CRITICAL
        % Emergency: rush to nearest available fuel
        destination = fpos(bestFuelIdx,:);
    elseif myFuel < FUEL_COMFORTABLE
        % Comfortable refuel
        destination = fpos(bestFuelIdx,:);
    elseif fuelAdvantage < ATTACK_FUEL_ADVANTAGE
        % Race opponent to the best fuel tank
        destination = fpos(bestFuelIdx,:);
    elseif myFuel >= opFuel
        % We're ahead, go attack
        destination = opPos;
    else
        % Outmatched but fuel exists — run to safest corner while looking for fuel
        destination = findSafestCorner(start, opPos, xyLim);
    end
end

% Clamp destination to stay inside the board and avoid wall-hit penalties
destination(1) = max(0.1, min(9.9, destination(1)));
destination(2) = max(0.1, min(9.9, destination(2)));

%% === STEP 4: Multi-mine avoidance (check ALL active mines, not just closest) ===
maxAvoidanceIterations = 3;
for iter = 1:maxAvoidanceIterations
    worstMine   = -1;
    worstDisc   = -inf;
    worstCpx    = [];
    worstCpy    = [];

    for i = 1:numMines
        if mineExist(i) == 1
            [col, cpx, cpy, disc] = checkLineCircle(start, destination, mpos(i,:), mineRad);
            if col && disc > worstDisc
                worstDisc = disc;
                worstMine = i;
                worstCpx  = cpx;
                worstCpy  = cpy;
            end
        end
    end

    if worstMine < 0
        break; % no collision, we're good
    end

    % Steer around the worst colliding mine
    midX    = (worstCpx(1) + worstCpx(2)) / 2;
    midY    = (worstCpy(1) + worstCpy(2)) / 2;
    awayVec = [midX - mpos(worstMine,1), midY - mpos(worstMine,2)];
    vecLen  = norm(awayVec);

    if vecLen > 1e-6
        awayVec = awayVec / vecLen;
    else
        % Mine is directly on path — dodge perpendicular to movement
        moveDir = destination - start;
        awayVec = [-moveDir(2), moveDir(1)];
        awayVec = awayVec / (norm(awayVec) + 1e-6);
    end

    destination = mpos(worstMine,:) + awayVec * (mineRad + MINE_SAFE_MARGIN);
    destination(1) = max(0.1, min(9.9, destination(1)));
    destination(2) = max(0.1, min(9.9, destination(2)));
end

%% === STEP 5: Compute final move ===
moveVec = destination - start;
moveMag = norm(moveVec);

if moveMag > 1e-6
    % Move at max speed, unless the target is closer than lmax
    move = moveVec / moveMag * min(moveMag, lmax);
else
    % We deliberately chose to stand still (or the target is exactly under us).
    % DO NOT charge the opponent. Conserve fuel.
    move = [0, 0];
end

%% ========== HELPER FUNCTIONS ==========

    function d = getDist(a, b)
        d = sqrt((a(1)-b(1))^2 + (a(2)-b(2))^2);
    end

    function s = minDistToMines(pos, mpos, mineExist, numMines)
        s = inf;
        for ii = 1:numMines
            if mineExist(ii) == 1
                s = min(s, getDist(pos, mpos(ii,:)));
            end
        end
        if isinf(s), s = 999; end
    end

    function corner = findSafestCorner(myPos, opPos, xyLim)
        corners = [xyLim(1,1), xyLim(2,1);
                   xyLim(1,1), xyLim(2,2);
                   xyLim(1,2), xyLim(2,1);
                   xyLim(1,2), xyLim(2,2)];
        bestCorner = corners(1,:);
        bestScore  = -inf;
        for k = 1:4
            myDist = getDist(myPos, corners(k,:));
            opDist = getDist(opPos, corners(k,:));
            % Prefer corners far from opponent and close to us
            score = opDist - myDist;
            if score > bestScore
                bestScore  = score;
                bestCorner = corners(k,:);
            end
        end
        corner = bestCorner;
    end

    function [collision, cpx, cpy, disc] = checkLineCircle(p1, p2, center, radius)
        dx  = p2(1) - p1(1);
        dy  = p2(2) - p1(2);
        len = sqrt(dx^2 + dy^2);

        collision = false;
        cpx  = [NaN NaN];
        cpy  = [NaN NaN];
        disc = -inf;

        if len < 1e-6, return; end

        dx = dx / len;
        dy = dy / len;
        fx = p1(1) - center(1);
        fy = p1(2) - center(2);

        a    = 1;
        b    = 2*(fx*dx + fy*dy);
        c    = fx^2 + fy^2 - radius^2;
        disc = b^2 - 4*a*c;

        if disc >= 0
            t1  = (-b - sqrt(disc)) / 2;
            t2  = (-b + sqrt(disc)) / 2;
            cpx = [p1(1)+t1*dx, p1(1)+t2*dx];
            cpy = [p1(2)+t1*dy, p1(2)+t2*dy];

            d1   = getDist(p1, [cpx(1), cpy(1)]);
            d2   = getDist([cpx(1), cpy(1)], p2);
            dTot = getDist(p1, p2);

            if dTot > d1 && dTot > d2
                collision = true;
            end
        end
    end

end