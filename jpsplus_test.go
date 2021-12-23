package jps_plus

import (
	"fmt"
	"strings"
	"testing"
)

var (
	grid50X50 [][]uint8
	grid5X10 [][]uint8
)

var trans = map[uint8]string{
	0: ".",
	1: "X",
	2: "o",
}

var world50X50 = `
.XX.........................XX....................
............................XX....................
.......XXX.........XX.......XX....................
.......XXX.........XX.......XX....................
.......XXX.........XX.......XX....................
.......XXX.........XX.......XX....................
...................XX.......XX....................
...................XXXXXXXXXXX....................
.............XXXXXXXX.............................
.............XXXXXXXX.............................
..................................................
..................................................
............................XX....................
.................X..........XX....................
.................X..........XX.....XXXXXXXXXXXXXXX
.................X..........XX....................
.................X..........XX....................
XXXXXXXXXXXXXXXXXX..........XX....................
............................XX....................
............................XXXXXXXXXXXXXX........
..................................................
..................................................
.................XXXXXXXXXXXXX....................
............................XX....................
.......XXX..................XX....................
.......XXX..................XX....................
.......XXX..................XX....................
.......XXX........................................
.......XXX........................................
.......XXX........................................
.......XXX................XXXXXXXXXXXXXXXXXXX.....
.......XXX................XX............XX........
.......XXX................XX............XX........
.......XXX................XX............XX........
.......XXX................XX............XX........
XXXXXXXXXXXXXXXX..........XX............XX........
..........................XX............XX........
..........................XX............XX........
..........................XX............XX........
..........................XX............XX........
..........................XX............XX........
..............XXXXXXXXXXXXXX............XX........
........................................XX........
...........................XXXXXXXXXXXXXXX........
...........................XX.....................
..........XXX..............XX.............XXXX....
..........XXX..............XX.............XX......
..........XXX..............XX.............XX......
..........XXX.............................XX......
..........XXX.............................XX......
`

var world5X10 = `
..X...X..
......X..
.XX...XX.
..X......
..X...X..
`

var (
	m5X10  AStarMatrix
	m50X50 AStarMatrix
)

func copyGrid(grid [][]uint8) [][]uint8 {
	temp := make([][]uint8, len(grid))
	copy(temp, grid)
	for i, v := range grid {
		copy(temp[i], v)
	}
	return temp
}

func TestMain(t *testing.M) {
	for _, row := range strings.Split(strings.TrimSpace(world50X50), "\n") {
		r := make([]uint8, 0, len(row))
		for _, raw := range row {
			n := 0
			if raw == 'X' {
				n = 1
			}
			r = append(r, uint8(n))
		}
		grid50X50 = append(grid50X50, r)
	}
	m50X50 = NewMatrix(grid50X50)
	for _, row := range strings.Split(strings.TrimSpace(world5X10), "\n") {
		r := make([]uint8, 0, len(row))
		for _, raw := range row {
			n := 0
			if raw == 'X' {
				n = 1
			}
			r = append(r, uint8(n))
		}
		grid5X10 = append(grid5X10, r)
	}
	m5X10 = NewMatrix(grid5X10)
	t.Run()
}

func TestJump(t *testing.T) {
	// "o" is the jump path
	path, err := m50X50.AStarJump([2]int64{0, 0}, [2]int64{49, 49})
	if err != nil {
		panic(err)
	}
	temp := copyGrid(grid50X50)
	for _, node := range path {
		temp[node.GetRow()][node.GetCol()] = 2
	}
	for _, v := range temp {
		for _, vv := range v {
			fmt.Printf("%s ", trans[vv])
		}
		fmt.Println()
	}
}

func BenchmarkJumpPlus5x10(b *testing.B) {
	b.ResetTimer()
	start := [2]int64{0, 0}
	end := [2]int64{0, 7}
	for i := 0; i < b.N; i++ {
		_, _ = m5X10.AStarJump(start, end)
	}
}

func BenchmarkJumpPlus50X50(b *testing.B) {
	b.ResetTimer()
	start := [2]int64{0, 0}
	end := [2]int64{49, 49}
	for i := 0; i < b.N; i++ {
		_, _ = m50X50.AStarJump(start, end)
	}
}
