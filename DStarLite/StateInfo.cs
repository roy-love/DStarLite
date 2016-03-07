using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DStarLite
{
    class StateInfo
    {
        /// <summary>
        /// As per [S. Koenig, 2002]
        /// </summary>
        private double[] keys = { double.PositiveInfinity, double.PositiveInfinity };
        private double g = double.PositiveInfinity;
        private double rhs = double.PositiveInfinity;
        private double cost = 1;
        private double cost_new;

        public double[] Keys
        {
            get
            {
                return keys;
            }

            set
            {
                keys = value;
            }
        }

        public double G
        {
            get
            {
                return g;
            }

            set
            {
                g = value;
            }
        }

        public double Rhs
        {
            get
            {
                return rhs;
            }

            set
            {
                rhs = value;
            }
        }

        public double Cost
        {
            get
            {
                return cost;
            }

            set
            {
                cost = value;
            }
        }

        public double Cost_new
        {
            get
            {
                return cost_new;
            }

            set
            {
                cost_new = value;
            }
        }
    }
}
