/*
	宏列表，批量处理标志位及相关函数
*/

#define FLAG_LIST(_)\
	_(F1)
	_(F2)

#define DEFINE_FLAG(flag) flag

enum flag{
	None=0;
	FLAG_LIST(DEFINE_FLAG)
	Total
};

#define FLAG_ACCESSOR(flag) \
	bool is##flag() const {\
		return hasFlags(1 << flag);\
	}\
	void set##flag() {\
		JS_ASSERT(!hasFlags(1 << flag));\
		setFlags(1<<flag);\
	}\
	void setNot##flag() {\
		JS_ASSERT(hasFlags(1 << flag));\
		removeFlags(1 << flag);\
	}
	FLAG_LIST(FLAG_ACCESSOR)
#undef FLAG_ACCESSOR