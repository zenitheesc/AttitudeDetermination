#ifndef ARRAY_HPP
#define ARRAY_HPP

#if __cplusplus >= 201703L
#define CONSTEXPR_17 constexpr
#else
#define CONSTEXPR_17
#endif


namespace alglin {
template<class Type, int N> class array {

	struct iterator {
		/* not "real" iterator */
		// using iterator_category = std::forward_iterator_tag;

		iterator(Type *ptr) : m_ptr(ptr) {}

		Type &operator*() const { return *m_ptr; }
		Type *operator->() { return m_ptr; }
		iterator &operator++() {
			m_ptr++;
			return *this;
		}
		iterator operator++(int) {
			iterator tmp = *this;
			++(*this);
			return tmp;
		}
		friend bool operator==(const iterator &a, const iterator &b) {
			return a.m_ptr == b.m_ptr;
		};
		friend bool operator!=(const iterator &a, const iterator &b) {
			return a.m_ptr != b.m_ptr;
		}

	  private:
		Type *m_ptr;
	};

  private:
	Type elem[N];

  public:
	array() = default;
	array(Type *from) {
		for (int i = 0; i < N; i++) { elem[i] = from[i]; }
	}
	CONSTEXPR_17 Type operator[](int i) const { return elem[i]; }
	CONSTEXPR_17 Type &operator[](int i) { return elem[i]; }
	Type *data() { return elem; }
	CONSTEXPR_17 iterator begin() { return iterator(&elem[0]); }
	CONSTEXPR_17 iterator end() {
		return iterator(&elem[N]);
	}// one pass the end
};
}// namespace alglin
#undef CONSTEXPR_17
#endif
