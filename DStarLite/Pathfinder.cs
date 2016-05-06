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
        double k_m = 0;
        int maxsteps = 8000;
        int steps = 0;
        bool change = false;
        List<State> changes = new List<State>();
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
            State temp = new State(x, y);

            if (S.ContainsKey(temp))
            {
                StateInfo cInfo = S[temp];
                if (steps != 0)
                {
                    cInfo.Cost_new = cost;
                    change = true;
                    changes.Add(temp);
                }
                else
                {
                    cInfo.Cost = cost;
                }
            }
        }
        /// <summary>
        /// As per [S. Koenig, 2002]
        /// </summary>
        public double[] calcKeys(State s)
        {
            double[] temp = new double[2];
            StateInfo sInfo = S[s];
            temp[0] = Math.Min(sInfo.G, sInfo.Rhs) + heuristics(start, s) + k_m;
            temp[1] = Math.Min(sInfo.G, sInfo.Rhs);
            return temp;
        }
        /// <summary>
        /// As per [S. Koenig, 2002]
        /// </summary>
        public void initialize()
        {
            U.Clear();
            k_m = 0;
            foreach (var s in S)
            {
                s.Value.Rhs = s.Value.G = double.PositiveInfinity;
                s.Value.Keys = new double[] { double.PositiveInfinity, double.PositiveInfinity };
            }
            StateInfo gInfo = S[goal];
            gInfo.Rhs = 0;
            gInfo.Keys[0] = heuristics(start, goal);
            gInfo.Keys[1] = 0;
            U.Add(goal);
            steps = 0;

        }
        /// <summary>
        /// As per [S. Koenig, 2002]
        /// </summary>
        public void updateVertex(State u)
        {
            StateInfo uInfo = S[u];
            if (uInfo.G != uInfo.Rhs && U.Contains(u))
            {
                uInfo.Keys = calcKeys(u);
            }
            else if (uInfo.G != uInfo.Rhs && !U.Contains(u))
            {
                uInfo.Keys = calcKeys(u);
                add(u);
            }
            else if (uInfo.G != uInfo.Rhs && U.Contains(u))
            {
                U.Remove(u);
            }
        }
        /// <summary>
        /// As per [S. Koenig, 2002], with one modification, maxsteps, because the code can loop forever otherwise.
        /// </summary>
        public void computerShotestPath()
        {
            StateInfo sInfo = S[start];
            while ((U.Any() && keyLessThan(U.First(), start) || sInfo.Rhs != sInfo.G) && steps < maxsteps)
            {
                steps++;
                State u = U.First();
                StateInfo uInfo = S[u];
                double[] k_old = uInfo.Keys;
                double[] k_new = calcKeys(u);
                if (keyLessThan(k_old, k_new))
                {
                    uInfo.Keys = k_new;
                }
                else if (uInfo.G > uInfo.Rhs)
                {
                    uInfo.G = uInfo.Rhs;
                    U.Remove(u);
                    List<State> tempList = Pred(u);
                    foreach (State s in tempList)
                    {
                        StateInfo info = S[s];
                        info.Rhs = Math.Min(info.Rhs, cost(s, u) + uInfo.G);
                        updateVertex(s);
                    }
                }
                else
                {
                    double g_old = uInfo.G;
                    uInfo.G = double.PositiveInfinity;
                    List<State> tempList = Pred(u);
                    tempList.Add(u);
                    foreach (State s in tempList)
                    {
                        StateInfo info = S[s];
                        if (info.Rhs == cost(s, u) + g_old)
                        {
                            if (s.X != goal.X && s.Y != goal.Y)
                            {
                                List<State> list = Succ(s);
                                if (list.Any())
                                {
                                    State smallest = list[0];
                                    StateInfo smallInfo = S[smallest];
                                    foreach (State ss in list)
                                    {
                                        StateInfo ssInfo = S[ss];
                                        if (cost(s, ss) + ssInfo.G < cost(s, smallest) + smallInfo.G)
                                        {
                                            smallest = ss;
                                            smallInfo = ssInfo;
                                        }
                                    }
                                    info.Rhs = smallInfo.G + cost(s, smallest);
                                }
                            }
                        }
                        updateVertex(s);
                    }
                }
            }
        }
        /// <summary>
        /// As per [S. Koenig, 2002]
        /// </summary>
        public void main()
        {
            State last = start;
            initialize();
            computerShotestPath();
            int h = 0;
            Console.WriteLine(start.X + " " + start.Y);
            while (start.X != goal.X || start.Y != goal.Y)
            {
                StateInfo startInfo = S[start];
                if (startInfo.G == double.PositiveInfinity)
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
                        if (cost(start, s) + sInfo.G < cost(start, smallest) + smallInfo.G)
                        {
                            smallest = s;
                            smallInfo = sInfo;
                        }
                    }
                    start = smallest;
                }
                Console.WriteLine(start.X + " " + start.Y);
                //For simulation and testing.
                if (h == 0)
                {
                    updateCost(2, 2, 3);
                    h++;
                }
                if (change)
                {
                    foreach (State temp in changes) {
                    k_m = k_m + heuristics(last, start);
                    last = start;
                    StateInfo changedInfo = S[temp];
                    double cc_old = changedInfo.Cost;
                    double cc_new = changedInfo.Cost_new;
                    changedInfo.Cost = changedInfo.Cost_new;
                        if (cc_old > cc_new)
                        {
                            changedInfo.Rhs = Math.Min(changedInfo.Rhs, cc_new + changedInfo.G);
                        }
                        else if (changedInfo.Rhs == cc_old + changedInfo.G)
                        {
                            if (temp.X != goal.X && temp.Y != goal.Y)
                            {
                                List<State> list2 = Succ(temp);
                                if (list2.Any())
                                {
                                    State smallest = list2[0];
                                    StateInfo smallInfo = S[smallest];
                                    foreach (State ss in list2)
                                    {
                                        StateInfo ssInfo = S[ss];
                                        if (cost(temp, ss) + ssInfo.G < cost(temp, smallest) + smallInfo.G)
                                        {
                                            smallest = ss;
                                            smallInfo = ssInfo;
                                        }
                                    }
                                    changedInfo.Rhs = smallInfo.G + cost(temp, smallest);
                                }
                            }
                        }
                        updateVertex(temp);
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

            if (aInfo.Keys[0] < bInfo.Keys[0])
            {
                return true;
            }
            if (aInfo.Keys[0] == bInfo.Keys[0])
            {
                if (aInfo.Keys[1] < bInfo.Keys[1])
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
            double min = Math.Abs(a.X - b.X);
            double max = Math.Abs(a.Y - a.Y);
            if (min > max)
            {
                temp = min;
                min = max;
                max = temp;
            }
            return ((m_sqrt2 - 1.0) * min + max);
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
                State temp = new State(s.X + directions[i, 0], s.Y + directions[i, 1]);

                if (S.ContainsKey(temp))
                {
                    tInfo = S[temp];
                    if (tInfo.Rhs > sInfo.Rhs)
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
                State temp = new State(s.X + directions[i, 0], s.Y + directions[i, 1]);

                if (S.ContainsKey(temp))
                {
                    tInfo = S[temp];
                    if (tInfo.G < sInfo.G)
                    {
                        tempList.Add(temp);
                    }

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
            return Math.Max(aInfo.Cost, bInfo.Cost);

        }
        double cost_new(State a, double cost)
        {
            StateInfo aInfo = S[a];
            return Math.Max(aInfo.Cost, cost);
        }
        public void add(State state)
        {
            if (U.Any())
            {
                for (int i = 0; i < U.Count; i++)
                {
                    State ss = U[i];

                    if (keyLessThan(state, ss))
                    {
                        U.Insert(i, state);
                        return;
                    }
                }
                U.Insert(U.Count, state);
            }
            else {
                U.Add(state);
            }
        }


    }
}