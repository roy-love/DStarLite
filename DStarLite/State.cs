using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DStarLite
{
    class State
    {
        private int x;
        private int y;

        /// <summary>
        /// The state is a combination of this class and StateInfo.
        /// </summary>
        /// <param name="x">The x coordinate.</param>
        /// <param name="y">The y coordinate.</param>
        public State(int x, int y)
        {
            this.X = x;
            this.Y = y;
        }

        public int X
        {
            get
            {
                return x;
            }

            set
            {
                x = value;
            }
        }

        public int Y
        {
            get
            {
                return y;
            }

            set
            {
                y = value;
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
            hash = 79 * hash + this.X;
            hash = 79 * hash + this.Y;
            return hash;
        }
    }
}