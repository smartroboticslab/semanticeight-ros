# Usage: objects_found mesh_comparison.log ...
objects_found() {
	if [ "$#" -eq 0 ]
	then
		printf '0\n'
	else
		sed -n -e 's/^Matched //' -e 's/ target meshes$//p' "$@" | awk '
		BEGIN { FS = "/" }
		NR == 1 { num_objects = $2 }
		{
			if ($2 != num_objects) {
				next
			}
			s += $1
			n++
		}
		END {
			if (n) {
				printf "Objects found: %.2f%%   (%.2f/%d)\n", 100*s/n/num_objects, s/n, num_objects
			} else {
				printf "Objects found: 0.00%%   (0.00/0)\n"
			}
		}
		'
	fi
}

# Usage: objects_found mesh_comparison.log ... | objects_found_to_pc
objects_found_to_pc() {
	sed -e 's/Objects found: //' -e 's/%.*$//'
}
