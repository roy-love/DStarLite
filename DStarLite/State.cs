namespace DStarLite
{
    internal class State
    {
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

        public int X { get; }

        public int Y { get; }

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
            var hash = 3;
            hash = 79 * hash + X;
            hash = 79 * hash + Y;
            return hash;
        }
    }
}