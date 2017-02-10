namespace DStarLite
{
    class StateInfo
    {
        /// <summary>
        /// As per [S. Koenig, 2002]
        /// </summary>
        private double[] _keys = { double.PositiveInfinity, double.PositiveInfinity };
        private double _g = double.PositiveInfinity;
        private double _rhs = double.PositiveInfinity;
        private double _cost = 1;
        private double _costNew;

        public double[] Keys
        {
            get
            {
                return _keys;
            }

            set
            {
                _keys = value;
            }
        }

        public double G
        {
            get
            {
                return _g;
            }

            set
            {
                _g = value;
            }
        }

        public double Rhs
        {
            get
            {
                return _rhs;
            }

            set
            {
                _rhs = value;
            }
        }

        public double Cost
        {
            get
            {
                return _cost;
            }

            set
            {
                _cost = value;
            }
        }

        public double Cost_new
        {
            get
            {
                return _costNew;
            }

            set
            {
                _costNew = value;
            }
        }
    }
}
