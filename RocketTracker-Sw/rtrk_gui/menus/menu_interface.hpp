#ifndef MENU_INTERFACE_H
#define MENU_INTERFACE_H

class IMenu {
public:
	virtual void deinit() = 0;
	virtual void init() = 0;

	virtual void render_gui(bool* open) = 0;
};

#endif