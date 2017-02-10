using System;
using System.Collections.Generic;
using System.Linq;

namespace DStarLite
{
    class Pathfinder
    {
        List<State> U = new List<State>();
        Dictionary<State, StateInfo> S = new Dictionary<State, StateInfo>();
        private State _start;
        State _goal;
        private double _kM;
        readonly int _maxsteps = 8000;
        private int _steps;
        private bool _change;
        List<State> changes = new List<State>();
        double m_sqrt2 = Math.Sqrt(2.0);
        int[,] directions = { { 0, -1 }, { 1, 0 }, { 0, 1 }, { -1, 0 }, { -1, -1 }, { 1, 1 }, { 1, -1 }, { -1, 1 } };

        public void SetGrid(int x, int y)
        {
            for (int i = 0; i < x; i++)
            {
                for (int j = 0; j < y; j++)
                {
                    S.Add(new State(i, j), new StateInfo());
                }
            }
        }

        public void SetStart(int x, int y)
        {
            State temp = new State(x, y);
            if (S.ContainsKey(temp))
            {
                _start = temp;
            }
        }

        public void SetGoal(int x, int y)
        {
            State temp = new State(x, y);
            if (S.ContainsKey(temp))
            {
                _goal = temp;
            }
        }

        public void UpdateCost(int x, int y, double cost)
        {
            State temp = new State(x, y);

            if (S.ContainsKey(temp))
            {
                StateInfo cInfo = S[temp];
                if (_steps != 0)
                {
                    cInfo.Cost_new = cost;
                    _change = true;
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
        public double[] CalcKeys(State s)
        {
            double[] temp = new double[2];
            StateInfo sInfo = S[s];
            temp[0] = Math.Min(sInfo.G, sInfo.Rhs) + Heuristics(_start, s) + _kM;
            temp[1] = Math.Min(sInfo.G, sInfo.Rhs);
            return temp;
        }
        /// <summary>
        /// As per [S. Koenig, 2002]
        /// </summary>
        public void Initialize()
        {
            U.Clear();
            _kM = 0;
            foreach (var s in S)
            {
                s.Value.Rhs = s.Value.G = double.PositiveInfinity;
                s.Value.Keys = new[] { double.PositiveInfinity, double.PositiveInfinity };
            }
            StateInfo gInfo = S[_goal];
            gInfo.Rhs = 0;
            gInfo.Keys[0] = Heuristics(_start, _goal);
            gInfo.Keys[1] = 0;
            U.Add(_goal);
            _steps = 0;

        }
        /// <summary>
        /// As per [S. Koenig, 2002]
        /// </summary>
        public void UpdateVertex(State u)
        {
            StateInfo uInfo = S[u];
            if (uInfo.G != uInfo.Rhs && U.Contains(u))
            {
                uInfo.Keys = CalcKeys(u);
            }
            else if (uInfo.G != uInfo.Rhs && !U.Contains(u))
            {
                uInfo.Keys = CalcKeys(u);
                Add(u);
            }
            else if (uInfo.G != uInfo.Rhs && U.Contains(u))
            {
                U.Remove(u);
            }
        }
        /// <summary>
        /// As per [S. Koenig, 2002], with one modification, maxsteps, because the code can loop forever otherwise.
        /// </summary>
        public void ComputerShotestPath()
        {
            StateInfo sInfo = S[_start];
            while ((U.Any() && KeyLessThan(U.First(), _start) || sInfo.Rhs != sInfo.G) && _steps < _maxsteps)
            {
                _steps++;
                State u = U.First();
                StateInfo uInfo = S[u];
                double[] kOld = uInfo.Keys;
                double[] kNew = CalcKeys(u);
                if (KeyLessThan(kOld, kNew))
                {
                    uInfo.Keys = kNew;
                }
                else if (uInfo.G > uInfo.Rhs)
                {
                    uInfo.G = uInfo.Rhs;
                    U.Remove(u);
                    List<State> tempList = Pred(u);
                    foreach (State s in tempList)
                    {
                        StateInfo info = S[s];
                        info.Rhs = Math.Min(info.Rhs, Cost(s, u) + uInfo.G);
                        UpdateVertex(s);
                    }
                }
                else
                {
                    double gOld = uInfo.G;
                    uInfo.G = double.PositiveInfinity;
                    List<State> tempList = Pred(u);
                    tempList.Add(u);
                    foreach (State s in tempList)
                    {
                        StateInfo info = S[s];
                        if (info.Rhs == Cost(s, u) + gOld)
                        {
                            if (s.X != _goal.X && s.Y != _goal.Y)
                            {
                                List<State> list = Succ(s);
                                if (list.Any())
                                {
                                    State smallest = list[0];
                                    StateInfo smallInfo = S[smallest];
                                    foreach (State ss in list)
                                    {
                                        StateInfo ssInfo = S[ss];
                                        if (Cost(s, ss) + ssInfo.G < Cost(s, smallest) + smallInfo.G)
                                        {
                                            smallest = ss;
                                            smallInfo = ssInfo;
                                        }
                                    }
                                    info.Rhs = smallInfo.G + Cost(s, smallest);
                                }
                            }
                        }
                        UpdateVertex(s);
                    }
                }
            }
        }
        /// <summary>
        /// As per [S. Koenig, 2002]
        /// </summary>
        public void Main()
        {
            State last = _start;
            Initialize();
            ComputerShotestPath();
            int h = 0;
            Console.WriteLine(_start.X + " " + _start.Y);
            while (_start.X != _goal.X || _start.Y != _goal.Y)
            {
                StateInfo startInfo = S[_start];
                if (startInfo.G == double.PositiveInfinity)
                {
                    Console.WriteLine("No path found.");
                    return;
                }
                List<State> tempList = Succ(_start);
                if (tempList.Any())
                {
                    State smallest = tempList[0];
                    StateInfo smallInfo = S[smallest];
                    foreach (State s in tempList)
                    {
                        StateInfo sInfo = S[s];
                        if (Cost(_start, s) + sInfo.G < Cost(_start, smallest) + smallInfo.G)
                        {
                            smallest = s;
                            smallInfo = sInfo;
                        }
                    }
                    _start = smallest;
                }
                Console.WriteLine(_start.X + " " + _start.Y);
                //For simulation and testing.
                if (h == 0)
                {
                    UpdateCost(2, 2, 3);
                    h++;
                }
                if (_change)
                {
                    foreach (State temp in changes) {
                    _kM = _kM + Heuristics(last, _start);
                    last = _start;
                    StateInfo changedInfo = S[temp];
                    double ccOld = changedInfo.Cost;
                    double ccNew = changedInfo.Cost_new;
                    changedInfo.Cost = changedInfo.Cost_new;
                        if (ccOld > ccNew)
                        {
                            changedInfo.Rhs = Math.Min(changedInfo.Rhs, ccNew + changedInfo.G);
                        }
                        else if (changedInfo.Rhs == ccOld + changedInfo.G)
                        {
                            if (temp.X != _goal.X && temp.Y != _goal.Y)
                            {
                                List<State> list2 = Succ(temp);
                                if (list2.Any())
                                {
                                    State smallest = list2[0];
                                    StateInfo smallInfo = S[smallest];
                                    foreach (State ss in list2)
                                    {
                                        StateInfo ssInfo = S[ss];
                                        if (Cost(temp, ss) + ssInfo.G < Cost(temp, smallest) + smallInfo.G)
                                        {
                                            smallest = ss;
                                            smallInfo = ssInfo;
                                        }
                                    }
                                    changedInfo.Rhs = smallInfo.G + Cost(temp, smallest);
                                }
                            }
                        }
                        UpdateVertex(temp);
                    }
                    _change = false;
                    ComputerShotestPath();
                }


            }

        }
        /// <summary>
        /// If you know A*, it shouldn't be a very big leap to understand what's going on here.
        /// </summary>
        /// <param name="a"> The first state.</param>
        /// <param name="b"> The second state.</param>
        /// <returns></returns>
        private bool KeyLessThan(State a, State b)
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
        bool KeyLessThan(double[] keyA, double[] keyB)
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
        private double Heuristics(State a, State b)
        {
            double temp;
            double min = Math.Abs(a.X - b.X);
            double max = Math.Abs(a.Y - b.Y);
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
        private double Cost(State a, State b)
        {

            StateInfo aInfo = S[a];
            StateInfo bInfo = S[b];
            return Math.Max(aInfo.Cost, bInfo.Cost);

        }

        public void Add(State state)
        {
            if (U.Any())
            {
                for (int i = 0; i < U.Count; i++)
                {
                    State ss = U[i];

                    if (KeyLessThan(state, ss))
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