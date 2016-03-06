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
        State topState;
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
            State temp = new State(x, y);
            if (S.ContainsKey(temp))
            {
                temp.cost = cost;
                StateInfo tempInfo = new StateInfo();
                S.Remove(temp);
                S.Add(temp, tempInfo);
                changed = temp;
                change = true;
            }
        }

        public double[] calcKeys(State s)
        {
            double[] temp = new double[2];
            StateInfo sInfo = S[s];
            temp[0] = Math.Min(sInfo.g, sInfo.rhs) + heuristics(start, s) + k_m;
            temp[1] = Math.Min(sInfo.g, sInfo.rhs);
            return temp;
        }

        public void initialize()
        {
            U.Clear();
            k_m = 0;
            foreach (var s in S)
            {
                s.Value.rhs = double.PositiveInfinity;
                s.Value.g = double.PositiveInfinity;
            }
            StateInfo gInfo = S[goal];
            gInfo.rhs = 0;
            gInfo.keys[0] = heuristics(start, goal);
            gInfo.keys[1] = 0;
            U.Add(goal);
            steps = 0;

        }

        public void updateVertex(State u)
        {
            StateInfo uInfo = S[u];
            if(uInfo.g != uInfo.rhs && U.Contains(u))
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

        public void computerShotestPath()
        {
            StateInfo sInfo = S[start];
            
            while ((U.Any() && keyLessThan(top(), start) || sInfo.rhs != sInfo.g) && steps < maxsteps)
            {
                steps++;
                StateInfo topInfo = S[topState];                
                double[] k_old = topInfo.keys;
                double[] k_new = calcKeys(topState);
                if (keyLessThan(k_old, k_new))
                {
                    topInfo.keys = k_new;
                }
                else if (topInfo.g > topInfo.rhs)
                {
                    topInfo.g = topInfo.rhs;
                    U.Remove(topState);
                    List<State> tempList = Pred(topState);
                    foreach (State s in tempList)
                    {
                        StateInfo info = S[s];
                        info.rhs = Math.Min(info.rhs, cost(s, topState)+topInfo.g);
                        updateVertex(s);
                    }
                }
                else
                {
                    double g_old = topInfo.g;
                    topInfo.g = double.PositiveInfinity;
                    List<State> tempList = Pred(topState);
                    tempList.Add(topState);
                    foreach (State s in tempList)
                    {
                        StateInfo info = S[s];
                        if (info.rhs == cost(s, topState) + g_old)
                        {
                            if(topState.x != goal.x && topState.y != goal.y)
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
                
                if (h == 1)
                {
                    updateCost(2, 2, double.PositiveInfinity);
                    h++;
                }
                if (change)
                {
                    k_m = k_m + heuristics(last, start);
                    last = start;
                    changeNeighbors();                    
                    change = false;
                    computerShotestPath();
                }


            }

        }

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

        public void changeNeighbors()
        {
            StateInfo sInfo = S[changed];
            for (int i = 0; i < 8; i++)
            {
                State temp = new State(changed.x + directions[i, 0], changed.y + directions[i, 1]);

                if (S.ContainsKey(temp))
                {
                        updateVertex(temp);
                }
            }
        }

        double cost(State a, State b)
        {
            return Math.Max(a.cost, b.cost);
        }


    }
}