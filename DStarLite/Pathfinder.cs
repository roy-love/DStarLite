using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DStarLite
{
    class Pathfinder
    {
        List<State> S = new List<State>();
        List<State> U = new List<State>();
        Dictionary<State, StateInfo> S2 = new Dictionary<State, StateInfo>();
        State start;
        State goal;
        State topState;
        int k_m = 0;
        int maxsteps = 8000;
        double m_sqrt2 = Math.Sqrt(2.0);
        int[,] directions = { { 0, -1 }, { 1, 0 }, { 0, 1 }, { -1, 0 }, { -1, -1 }, { 1, 1 }, { 1, -1 }, { -1, 1 } };

        public void setGrid(int x, int y)
        {
            for (int i = 0; i < x; i++)
            {
                for (int j = 0; j < y; j++)
                {
                    S2.Add(new State(i, j), new StateInfo());
                }
            }
        }

        public void setStart(int x, int y)
        {
            State temp = new State(x, y);
            if (S2.ContainsKey(temp))
            {
                start = temp;
            }
        }

        public void setGoal(int x, int y)
        {
            State temp = new State(x, y);
            if (S2.ContainsKey(temp))
            {
                goal = temp;
            }
        }

        public double[] calcKeys(State s)
        {
            double[] temp = new double[2];
            StateInfo sInfo = S2[s];
            temp[0] = Math.Min(sInfo.g, sInfo.rhs) + heuristics(start, s);
            temp[1] = Math.Min(sInfo.g, sInfo.rhs);
            return temp;
        }

        public void initialize()
        {
            U.Clear();
            foreach (var s in S2)
            {
                s.Value.rhs = double.PositiveInfinity;
                s.Value.g = double.PositiveInfinity;
            }
            StateInfo gInfo = S2[goal];
            gInfo.rhs = 0;
            gInfo.key_0 = heuristics(start, goal);
            gInfo.key_1 = 0;
            U.Add(goal);

        }

        public void updateVertex(State u)
        {
            StateInfo uInfo = S2[u];
            if (u != start)
            {
                List<State> tempList = Succ(u);
                if (tempList.Any())
                {
                    State smallest = tempList[0];
                    StateInfo smallInfo = S2[smallest];
                    foreach (State s in tempList)
                    {
                        StateInfo sInfo = S2[s];
                        if (cost(s, u) + sInfo.g < cost(smallest, u) + smallInfo.g)
                        {
                            smallest = s;
                            smallInfo = sInfo;
                        }
                    }
                    uInfo.rhs = smallInfo.g + cost(u, smallest);
                }

            }
            if (U.Contains(u))
            {
                U.Remove(u);
            }
            if (uInfo.g != uInfo.rhs)
            {
                U.Add(u);
                double[] keys = calcKeys(u);
                uInfo.key_0 = keys[0];
                uInfo.key_1 = keys[1];
            }
        }

        public void computerShotestPath()
        {
            StateInfo sInfo = S2[start];
            int steps = 0;
            while ((U.Any() && keyLessThan(top(), start) || sInfo.rhs != sInfo.g) && steps < maxsteps)
            {
                steps++;
                StateInfo topInfo = S2[topState];
                if (topInfo.g > topInfo.rhs)
                {
                    topInfo.g = topInfo.rhs;

                    List<State> tempList = Pred(topState);
                    foreach (State s in tempList)
                    {
                        updateVertex(s);
                    }
                    updateVertex(topState);
                }
                else
                {
                    topInfo.g = double.PositiveInfinity;
                    updateVertex(topState);
                    List<State> tempList = Pred(topState);
                    foreach (State s in tempList)
                    {
                        updateVertex(s);
                    }
                }
            }
        }

        public void main()
        {
            initialize();
            computerShotestPath();

        }

        bool keyLessThan(State a, State b)
        {
            StateInfo aInfo = S2[a];
            StateInfo bInfo = S2[b];

            if (aInfo.key_0 < bInfo.key_0)
            {
                return true;
            }
            if (aInfo.key_0 == bInfo.key_0)
            {
                if (aInfo.key_1 < bInfo.key_1)
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
            topState = temp;
            return temp;
        }

        List<State> Pred(State s)
        {
            List<State> tempList = new List<State>();
            StateInfo sInfo = S2[s];
            StateInfo tInfo;
            for (int i = 0; i < 8; i++)
            {
                State temp = new State(s.x + directions[i, 0], s.y + directions[i, 1]);

                if (S2.ContainsKey(temp))
                {
                    tInfo = S2[temp];
                    if (tInfo.rhs > sInfo.rhs)
                    {
                        tempList.Add(temp);
                    }

                }
            }
            return tempList;
        }
        List<State> Succ(State s)
        {
            List<State> tempList = new List<State>();
            StateInfo sInfo = S2[s];
            StateInfo tInfo;
            for (int i = 0; i < 8; i++)
            {
                State temp = new State(s.x + directions[i, 0], s.y + directions[i, 1]);

                if (S2.ContainsKey(temp))
                {
                    tInfo = S2[temp];
                    if (tInfo.g < sInfo.g)
                    {
                        tempList.Add(temp);
                    }

                }
            }
            return tempList;
        }

        double cost(State a, State b)
        {
            return Math.Max(a.cost, b.cost);
        }


    }
}