namespace DStarLite
{
    class State
    {
        private int _x;
        private int _y;

        /// <summary>
        /// The state is a combination of this class and StateInfo.
        /// </summary>
        /// <param name="x">The x coordinate.</param>
        /// <param name="y">The y coordinate.</param>
        public State(int x, int y)
        {
            X = x;
            Y = y;
        }

        public int X
        {
            get
            {
                return _x;
            }

            set
            {
                _x = value;
            }
        }

        public int Y
        {
            get
            {
                return _y;
            }

            set
            {
                _y = value;
            }
        }

        public override bool Equals(object obj)
        {
            if (this == obj)
            {
                return true;
            }
            var item = obj as State;
            if (item == null)
            {
                return false;
            }
            return (X == item.X && Y == item.Y);
        }

        public override int GetHashCode()
        {
            int hash = 3;
            hash = 79 * hash + X;
            hash = 79 * hash + Y;
            return hash;
        }
    }
}