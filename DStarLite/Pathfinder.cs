using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DStarLite
{
    class Pathfinder
    {
        List<State> U = new List<State>();
        Dictionary<State, StateInfo> S = new Dictionary<State, StateInfo>();
        State start;
        State goal;
        State u;
        double k_m = 0;
        int maxsteps = 8000;
        int steps = 0;
        bool change = false;
        State changed;
        double m_sqrt2 = Math.Sqrt(2.0);
        int[,] directions = { { 0, -1 }, { 1, 0 }, { 0, 1 }, { -1, 0 }, { -1, -1 }, { 1, 1 }, { 1, -1 }, { -1, 1 } };

        public void setGrid(int x, int y)
        {
            for (int i = 0; i < x; i++)
            {
                for (int j = 0; j < y; j++)
                {
                    S.Add(new State(i, j), new StateInfo());
                }
            }
        }

        public void setStart(int x, int y)
        {
            State temp = new State(x, y);
            if (S.ContainsKey(temp))
            {
                start = temp;
            }
        }

        public void setGoal(int x, int y)
        {
            State temp = new State(x, y);
            if (S.ContainsKey(temp))
            {
                goal = temp;
            }
        }

        public void updateCost(int x, int y, double cost)
        {
            changed = new State(x, y);

            if (S.ContainsKey(changed))
            {
                StateInfo cInfo = S[changed];
                cInfo.cost_new = cost;
                change = true;
            }
        }
        /// <summary>
        /// As shown in the paper.
        /// </summary>
        /// <param name="s"></param>
        /// <returns></returns>
        public double[] calcKeys(State s)
        {
            double[] temp = new double[2];
            StateInfo sInfo = S[s];
            temp[0] = Math.Min(sInfo.g, sInfo.rhs) + heuristics(start, s) + k_m;
            temp[1] = Math.Min(sInfo.g, sInfo.rhs);
            return temp;
        }
        /// <summary>
        /// As shown in the paper.
        /// </summary>
        public void initialize()
        {
            U.Clear();
            k_m = 0;
            foreach (var s in S)
            {
                s.Value.rhs = s.Value.g = double.PositiveInfinity;
                //s.Value.g = double.PositiveInfinity;
            }
            StateInfo gInfo = S[goal];
            gInfo.rhs = 0;
            gInfo.keys[0] = heuristics(start, goal);
            gInfo.keys[1] = 0;
            U.Add(goal);
            steps = 0;

        }
        /// <summary>
        /// As shown in the paper.
        /// </summary>
        /// <param name="u"></param>
        public void updateVertex(State u)
        {
            StateInfo uInfo = S[u];
            if (uInfo.g != uInfo.rhs && U.Contains(u))
            {
                uInfo.keys = calcKeys(u);
            }
            else if (uInfo.g != uInfo.rhs && !U.Contains(u))
            {
                uInfo.keys = calcKeys(u);
                U.Add(u);
            }
            else if (uInfo.g != uInfo.rhs && U.Contains(u))
            {
                U.Remove(u);
            }
        }
        /// <summary>
        /// As shown in the paper.
        /// </summary>
        public void computerShotestPath()
        {
            StateInfo sInfo = S[start];

            while ((U.Any() && keyLessThan(top(), start) || sInfo.rhs != sInfo.g) && steps < maxsteps)
            {
                steps++;
                StateInfo uInfo = S[u];
                double[] k_old = uInfo.keys;
                double[] k_new = calcKeys(u);
                if (keyLessThan(k_old, k_new))
                {
                    uInfo.keys = k_new;
                }
                else if (uInfo.g > uInfo.rhs)
                {
                    uInfo.g = uInfo.rhs;
                    U.Remove(u);
                    List<State> tempList = Pred(u);
                    foreach (State s in tempList)
                    {
                        StateInfo info = S[s];
                        info.rhs = Math.Min(info.rhs, cost(s, u) + uInfo.g);
                        updateVertex(s);
                    }
                }
                else
                {
                    double g_old = uInfo.g;
                    uInfo.g = double.PositiveInfinity;
                    List<State> tempList = Pred(u);
                    tempList.Add(u);
                    foreach (State s in tempList)
                    {
                        StateInfo info = S[s];
                        if (info.rhs == cost(s, u) + g_old)
                        {
                            if (u.x != goal.x && u.y != goal.y)
                            {
                                List<State> list = Succ(s);
                                if (tempList.Any())
                                {
                                    State smallest = tempList[0];
                                    StateInfo smallInfo = S[smallest];
                                    foreach (State ss in tempList)
                                    {
                                        StateInfo ssInfo = S[ss];
                                        if (cost(s, ss) + ssInfo.g < cost(s, smallest) + smallInfo.g)
                                        {
                                            smallest = ss;
                                            smallInfo = ssInfo;
                                        }
                                    }
                                    info.rhs = smallInfo.g + cost(s, smallest);
                                }
                            }
                        }
                        updateVertex(s);
                    }
                }
            }
        }
        /// <summary>
        /// You guessed it, in the paper.
        /// </summary>
        public void main()
        {
            State last = start;
            initialize();
            computerShotestPath();
            int h = 0;
            Console.WriteLine(start.x + " " + start.y);
            while (start.x != goal.x && start.y != goal.y)
            {
                StateInfo startInfo = S[start];
                if (startInfo.g == double.PositiveInfinity)
                {
                    Console.WriteLine("No path found.");
                    return;
                }
                List<State> tempList = Succ(start);
                if (tempList.Any())
                {
                    State smallest = tempList[0];
                    StateInfo smallInfo = S[smallest];
                    foreach (State s in tempList)
                    {
                        StateInfo sInfo = S[s];
                        if (cost(start, s) + sInfo.g < cost(start, smallest) + smallInfo.g)
                        {
                            smallest = s;
                            smallInfo = sInfo;
                        }
                    }
                    start = smallest;
                }
                Console.WriteLine(start.x + " " + start.y);
                // For simulation and testing.
                if (h == 0)
                {
                    updateCost(2, 2, 3);
                    h++;
                }
                if (change)
                {
                    k_m = k_m + heuristics(last, start);
                    last = start;
                    List<State> list = changeNeighbors();
                    double c_old;
                    double c_new;
                    StateInfo changedInfo = S[changed];
                    double cc_old = changedInfo.cost;
                    double cc_new = changedInfo.cost_new;
                    changedInfo.cost = changedInfo.cost_new;
                    foreach (State s in list)
                    {
                        StateInfo info = S[s];
                        c_old = cost_new(s, cc_old);
                        c_new = cost_new(s, cc_new);

                        if (c_old > c_new)
                        {
                            info.rhs = Math.Min(info.rhs, c_new + changedInfo.g);
                        }
                        else if (info.rhs == c_old + changedInfo.g)
                        {
                            if (s.x != goal.x && s.y != goal.y)
                            {
                                List<State> list2 = Succ(s);
                                if (list2.Any())
                                {
                                    State smallest = list2[0];
                                    StateInfo smallInfo = S[smallest];
                                    foreach (State ss in list2)
                                    {
                                        StateInfo ssInfo = S[ss];
                                        if (cost(s, ss) + ssInfo.g < cost(s, smallest) + smallInfo.g)
                                        {
                                            smallest = ss;
                                            smallInfo = ssInfo;
                                        }
                                    }
                                    info.rhs = smallInfo.g + cost(s, smallest);
                                }
                            }
                        }
                        updateVertex(s);
                    }
                    change = false;
                    computerShotestPath();
                }


            }

        }
        /// <summary>
        /// If you know A*, it shouldn't be a very big leap to understand what's going on here.
        /// </summary>
        /// <param name="a"> The first state.</param>
        /// <param name="b"> The second state.</param>
        /// <returns></returns>
        bool keyLessThan(State a, State b)
        {
            StateInfo aInfo = S[a];
            StateInfo bInfo = S[b];

            if (aInfo.keys[0] < bInfo.keys[0])
            {
                return true;
            }
            if (aInfo.keys[0] == bInfo.keys[0])
            {
                if (aInfo.keys[1] < bInfo.keys[1])
                {
                    return true;
                }
            }

            return false;
        }
        /// <summary>
        /// Same as the above, but with keys instead of states.
        /// </summary>
        /// <param name="keyA">First pair.</param>
        /// <param name="keyB">Second pair.</param>
        /// <returns></returns>
        bool keyLessThan(double[] keyA, double[] keyB)
        {
            if (keyA[0] < keyB[0])
            {
                return true;
            }
            if (keyA[0] == keyB[0])
            {
                if (keyA[1] < keyB[1])
                {
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// The method used to estimate the distance from one state to another.
        /// </summary>
        /// <param name="a">The first state.</param>
        /// <param name="b">The second state.</param>
        /// <returns>Returns the heuristics value.</returns>
        double heuristics(State a, State b)
        {
            double temp;
            double min = Math.Abs(a.x - b.x);
            double max = Math.Abs(a.y - a.y);
            if (min > max)
            {
                temp = min;
                min = max;
                max = temp;
            }
            return ((m_sqrt2 - 1.0) * min + max);
        }
        /// <summary>
        /// Gets the state with the highest priority ('till I implement an actual priority queue).
        /// </summary>
        /// <returns>State, highest priority in U.</returns>
        State top()
        {
            State temp = U[0];
            foreach (State s in U)
            {
                if (keyLessThan(s, temp))
                {
                    temp = s;
                }
            }
            u = temp;
            return temp;
        }
        /// <summary>
        /// Gets the predecessors of a state s.
        /// </summary>
        /// <param name="s">The state.</param>
        /// <returns>A list of predecessors, may be empty.</returns>
        List<State> Pred(State s)
        {
            List<State> tempList = new List<State>();
            StateInfo sInfo = S[s];
            StateInfo tInfo;
            for (int i = 0; i < 8; i++)
            {
                State temp = new State(s.x + directions[i, 0], s.y + directions[i, 1]);

                if (S.ContainsKey(temp))
                {
                    tInfo = S[temp];
                    if (tInfo.rhs > sInfo.rhs)
                    {
                        tempList.Add(temp);
                    }

                }
            }
            return tempList;
        }
        /// <summary>
        /// Gets the successors of a state s.
        /// </summary>
        /// <param name="s">The state.</param>
        /// <returns>A list of successors, may be empty.</returns>
        List<State> Succ(State s)
        {
            List<State> tempList = new List<State>();
            StateInfo sInfo = S[s];
            StateInfo tInfo;
            for (int i = 0; i < 8; i++)
            {
                State temp = new State(s.x + directions[i, 0], s.y + directions[i, 1]);

                if (S.ContainsKey(temp))
                {
                    tInfo = S[temp];
                    if (tInfo.g < sInfo.g)
                    {
                        tempList.Add(temp);
                    }

                }
            }
            return tempList;
        }
        /// <summary>
        /// Gets adjacent states to the State instance 'changed'.
        /// </summary>
        public List<State> changeNeighbors()
        {
            List<State> tempList = new List<State>();
            StateInfo sInfo = S[changed];
            for (int i = 0; i < 8; i++)
            {
                State temp = new State(changed.x + directions[i, 0], changed.y + directions[i, 1]);

                if (S.ContainsKey(temp))
                {
                    tempList.Add(temp);
                }
            }
            return tempList;
        }

        /// <summary>
        /// Gets the cost of traversal from one state to another adjacent state.
        /// </summary>
        /// <param name="a">First state.</param>
        /// <param name="b">Second state.</param>
        /// <returns>The cost, double.</returns>
        double cost(State a, State b)
        {

            StateInfo aInfo = S[a];
            StateInfo bInfo = S[b];
            return Math.Max(aInfo.cost, bInfo.cost);

        }
        double cost_new(State a, double cost)
        {
            StateInfo aInfo = S[a];
            return Math.Max(aInfo.cost, cost);
        }


    }
}