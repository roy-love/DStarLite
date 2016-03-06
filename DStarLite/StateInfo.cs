using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DStarLite
{
    class StateInfo
    {
        public double[] keys = { double.PositiveInfinity, double.PositiveInfinity };
        public double g = double.PositiveInfinity;
        public double rhs = double.PositiveInfinity;
    }
}
