package jps_plus

type Node int64

func newNode(row, col int64) Node {
	r := row<<32 | col
	return Node(r)
}

func (n Node) GetRow() int64 {
	return int64(n) >> 32
}

func (n Node) GetCol() int64 {
	return int64(int32(n))
}

func (n Node) Arr() [2]int64 {
	return [2]int64{n.GetRow(), n.GetCol()}
}

func (n Node) equals(otherNode Node) bool {
	return n == otherNode
}
